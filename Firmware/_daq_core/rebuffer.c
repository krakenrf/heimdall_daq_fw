/*
 * 
 * Description :
 * Implements rebuffering on the data acquistion chain. The main usecase
 * of the processing block is to change the iq sample blocksize-buffersize.
 * 
 * Project : HeIMDALL DAQ Firmware
 * Author  : Tamas Peto
 * License : GNU GPL V3
 *
 * Copyright (C) 2018-2021  Tamás Pető
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/* 
 * Implementation note:
 *  All internal *buffer_size variables are interpreted in IQ samples 
 *  Eg: When the buffer_size has a value of 2**18, then 2**18 IQ sample is actually handled per channel
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include "rtl_daq.h"
#include "log.h"
#include "ini.h"
#include "iq_header.h"
#include "sh_mem_util.h"

#define INI_FNAME "daq_chain_config.ini"
#define FATAL_ERR(l) log_fatal(l); return -1;
/*
 * This structure stores the configuration parameters, 
 * that are loaded from the ini file
 */ 
typedef struct
{
    int num_ch;
    int cal_size;
    int cpi_size;
    int daq_buffer_size;
    int decimation_ratio;    
    int log_level;      
} configuration;

/*
 * Ini configuration parser callback function  
*/
static int handler(void* conf_struct, const char* section, const char* name,
                   const char* value)
{
    configuration* pconfig = (configuration*) conf_struct;

    #define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    if (MATCH("hw", "num_ch"))
    {pconfig->num_ch = atoi(value);}
    else if (MATCH("calibration", "corr_size"))
    {pconfig->cal_size = atoi(value);}
    else if (MATCH("daq", "daq_buffer_size"))
    {pconfig->daq_buffer_size = atoi(value);}
    else if (MATCH("pre_processing", "decimation_ratio")) 
    {pconfig->decimation_ratio = atoi(value);}
    else if (MATCH("pre_processing", "cpi_size")) 
    {pconfig->cpi_size = atoi(value);}
    else if (MATCH("daq", "log_level"))
    {pconfig->log_level = atoi(value);}
    else {return 0;  /* unknown section/name, error */}
    return 0;
}

int main(int argc, char* argv[])
/*
 *
 * Parameters:
 * -----------
 * argv[1]: Drop mode [int]
 * 
 */
{    
    log_set_level(LOG_TRACE);
    configuration config;    
        
    int exit_flag=0;
    int ch_num;
    int succ;
    int read_size; // Used in general to store the number of succesfully read bytes
    
    // Used for the data buffering
    int in_buffer_size, out_buffer_size, cal_out_buffer_size, active_out_buffer_size; // Interpreted in IQ samples (1 sample -> 2 uint8)
    int buffer_num; // Size of the circular buffer (measured in input buffer size)
    int chunk_size, chunk_size_2; // Counts uint8_t values (1 sample -> 2 uint8_t)    
    struct circ_buffer_struct* circ_buff_structs;
    int rd_offset=0, wr_offset=0, available=0;
    size_t offset = 0;
    // Used for managing the data frames
    uint32_t expected_frame_index=-1;    
    void* frame_ptr;
    struct iq_header_struct* iq_header = calloc(1, sizeof(struct iq_header_struct));    
    // Used for the shared memory interface
    int active_buff_ind = 0;
    bool drop_mode = true;

    uint32_t adc_overdrive_flags=0; // Used to accumulate the overdrive flags in a CPI
    
    /* Set drop mode from the command prompt*/    
    if (argc == 2){drop_mode = atoi(argv[1]);}

    /* Set parameters from the config file*/
    if (ini_parse(INI_FNAME, handler, &config) < 0) {
        log_fatal("Configuration could not be loaded, exiting ..");
        return -2;
    }   
    in_buffer_size  = config.daq_buffer_size;
    out_buffer_size = config.cpi_size * config.decimation_ratio;
    cal_out_buffer_size = config.cal_size; 
    active_out_buffer_size = 0;
    ch_num = config.num_ch;
    log_set_level(config.log_level);          
    log_info("Config succesfully loaded from %s",INI_FNAME);
    log_info("Channel number: %d", ch_num);
    log_info("Input buffer size: %d IQ samples per channel", in_buffer_size);
    log_info("Output buffer size: %d IQ samples per channel", out_buffer_size);
    log_info("Calibration buffer size: %d IQ samples per channel", cal_out_buffer_size);

    // Determine the neccesary size of the circular buffers
    int buffer_num_data = out_buffer_size / in_buffer_size + 2;
    int buffer_num_cal  = cal_out_buffer_size / in_buffer_size + 2;
    if (buffer_num_data >= buffer_num_cal) buffer_num = buffer_num_data;
    else buffer_num = buffer_num_cal;     
    log_info("Buffer no: %d", buffer_num);
    
    // Allocation and initialization of the circular buffers
    circ_buff_structs = malloc(ch_num*sizeof(*circ_buff_structs));
    for(int m=0;m<ch_num;m++)
    {                
        circ_buff_structs[m].iq_circ_buffer = calloc(buffer_num * in_buffer_size*2, sizeof(uint8_t)); // *2-> I and Q
    }
    
    /* Initializing output shared memory interface */
    struct shmem_transfer_struct* output_sm_buff = calloc(1, sizeof(struct shmem_transfer_struct));
    if (out_buffer_size >= cal_out_buffer_size)
    {
        output_sm_buff->shared_memory_size = out_buffer_size*ch_num*sizeof(uint8_t)*2+IQ_HEADER_LENGTH;
    }
    else
    {
        output_sm_buff->shared_memory_size = cal_out_buffer_size*ch_num*sizeof(uint8_t)*2+IQ_HEADER_LENGTH;
    }
    output_sm_buff->io_type = 0; // Output type
    output_sm_buff->drop_mode = drop_mode;
    strcpy(output_sm_buff->shared_memory_names[0], DECIMATOR_IN_SM_NAME_A);
    strcpy(output_sm_buff->shared_memory_names[1], DECIMATOR_IN_SM_NAME_B);
    strcpy(output_sm_buff->fw_ctr_fifo_name, DECIMATOR_IN_FW_FIFO);
    strcpy(output_sm_buff->bw_ctr_fifo_name, DECIMATOR_IN_BW_FIFO);

    succ = init_out_sm_buffer(output_sm_buff);
    if(succ !=0){FATAL_ERR("Shared memory initialization failed")}
	
    /*
     *
     * ---> Main Processing Loop <---
     *
     */
    while(!exit_flag)
    {
        CHK_DATA_PIPE(stdin);
        /*
         *------------------
         *  IQ Frame Reading
         *------------------
        */        
        /* Reading IQ header */
        read_size = fread(iq_header, sizeof(struct iq_header_struct), 1, stdin);                
        //dump_iq_header(iq_header); // Uncomment to debug IQ header content
        CHK_FR_READ(read_size,1);
        CHK_SYNC_WORD(check_sync_word(iq_header));
        log_trace("<-- Frame received: type: %d, daq ind:[%d]",iq_header->frame_type, iq_header->daq_block_index);
        if (expected_frame_index == -1)
        {expected_frame_index = iq_header->daq_block_index;}

        if (expected_frame_index != iq_header->daq_block_index)
        {
            log_warn("Frame index missmatch. Expected %d <--> %d Received",expected_frame_index,iq_header->daq_block_index);
            expected_frame_index = iq_header->daq_block_index;
        }
        expected_frame_index += 1;
        
        /* Reading multichannel IQ data */
        if (iq_header->cpi_length > 0)
        {
            for(int m=0;m<iq_header->active_ant_chs;m++)
            {   
                // Get the circular buffer structure of the mth channel 
                struct circ_buffer_struct *cbuff_m = &circ_buff_structs[m];
                
                // Read into the buffer            
                read_size = fread(cbuff_m->iq_circ_buffer+rd_offset, sizeof(uint8_t), (in_buffer_size*2), stdin);
                CHK_FR_READ(read_size, in_buffer_size*2);                                                                  
            }
        }
        /*
         *---------------------------------------------------
         *  Data handling depending on the current frame type 
         *---------------------------------------------------
        */
        switch(iq_header->frame_type)
        {   
            case FRAME_TYPE_DUMMY:
            {         
                /* Reset module parameters */
                for(int m=0;m<ch_num;m++)
                {                
                    struct circ_buffer_struct *cbuff_m = &circ_buff_structs[m];
                    memset(circ_buff_structs[m].iq_circ_buffer, 0, buffer_num * in_buffer_size*2*sizeof(*cbuff_m->iq_circ_buffer));
                }
                wr_offset = 0;
                rd_offset = 0;  
                available = 0;                
            }
            break;
            case FRAME_TYPE_DATA:
            case FRAME_TYPE_CAL:
            {
                // Update read offset
                rd_offset += (in_buffer_size*2);
                rd_offset = rd_offset%(buffer_num * in_buffer_size*2);                        
                
                // Update the available data field after succesfull read 
                available += read_size;                

                // Accumulate ADC overdrive flags
                adc_overdrive_flags |= iq_header->adc_overdrive_flags;
            }
            break;
            default:
                log_error("Unidentified frame type, dropping..");
            break;
        }        
        /*
         *------------------
         *  IQ Frame Writing
         *------------------
        */ 
        if (iq_header->frame_type == FRAME_TYPE_DUMMY)
        {
            /*Acquire buffer from the sink block*/
            active_buff_ind = wait_buff_free(output_sm_buff);            
            switch(active_buff_ind)
            { 
                case 0:
                case 1:
                    active_out_buffer_size=0;
                    frame_ptr = output_sm_buff->shm_ptr[active_buff_ind];                        
                    
                    iq_header->cpi_length = 0;
                    
                    /* Place IQ header into the output buffer*/
                    memcpy(frame_ptr, iq_header,1024);
                    send_ctr_buff_ready(output_sm_buff, active_buff_ind);
                    log_trace("--> Transfering frame: type: %d, daq ind:[%d]",iq_header->frame_type, iq_header->daq_block_index);
		    break;
                case 3: // Frame drop                    
                    break;
                default:
                    log_error("Failed to acquire free buffer");
                    exit_flag = 1;
            }
        }
        else if ( (iq_header->frame_type == FRAME_TYPE_CAL) & (available >= (cal_out_buffer_size*2)))
        {active_out_buffer_size = cal_out_buffer_size;}
        else if ( (iq_header->frame_type == FRAME_TYPE_DATA) & (available >= (out_buffer_size*2)))
        {active_out_buffer_size = out_buffer_size;}
        else{active_out_buffer_size=0;}

        if (active_out_buffer_size) // Data has been accumulated (Either for cal frame or data frame)            
        {
            /*Acquire buffer from the sink block*/
            active_buff_ind = wait_buff_free(output_sm_buff);  
            log_debug("Acquired free buffer: %d",active_buff_ind);
            switch(active_buff_ind)
            { 
                case 0:
                case 1:                
                    frame_ptr = output_sm_buff->shm_ptr[active_buff_ind];                        
                    
                    /* Place IQ header into the output buffer*/
                    iq_header->cpi_length = active_out_buffer_size;
                    iq_header->adc_overdrive_flags = adc_overdrive_flags;

                    float timestamp_adjust = (float) (available-active_out_buffer_size*2)/2*1000/iq_header->sampling_freq;                    
                    log_debug("Timestamp adjust: %f ms", timestamp_adjust);
                    iq_header->time_stamp -= (int) round(timestamp_adjust);                    
                    adc_overdrive_flags = 0;
                    memcpy(frame_ptr, iq_header,1024);
                    
                    /* Place Multichannel IQ data */
                    chunk_size = buffer_num * in_buffer_size * 2 - wr_offset; // Available data until the end of the circular buffer                     
                    if (chunk_size >= active_out_buffer_size*2)
                    {
                        for(int m=0;m<iq_header->active_ant_chs;m++)
                        {   
                            // Get the circular buffer structure of the mth channel 
                            struct circ_buffer_struct *cbuff_m = &circ_buff_structs[m];
                            offset = IQ_HEADER_LENGTH/(sizeof(uint8_t)) + m*active_out_buffer_size*2;
                            memcpy(frame_ptr+offset, cbuff_m->iq_circ_buffer+wr_offset, active_out_buffer_size*2);
                        }
                        wr_offset += active_out_buffer_size*2;
                        wr_offset = wr_offset % (buffer_num * in_buffer_size*2);                    
                    }
                    else
                    {                
                        chunk_size_2 = active_out_buffer_size*2-chunk_size;
                        for(int m=0;m<iq_header->active_ant_chs;m++)
                        {   
                            // Get the circular buffer structure of the mth channel 
                            struct circ_buffer_struct *cbuff_m = &circ_buff_structs[m];
                            offset = IQ_HEADER_LENGTH/(sizeof(uint8_t)) + m*active_out_buffer_size*2;
                            memcpy(frame_ptr+offset, cbuff_m->iq_circ_buffer+wr_offset, chunk_size);
                            memcpy(frame_ptr+offset+chunk_size, cbuff_m->iq_circ_buffer, chunk_size_2);
                        }
                        wr_offset = chunk_size_2;
                    }   
                    available -= active_out_buffer_size*2;                
                    send_ctr_buff_ready(output_sm_buff, active_buff_ind);                                      
                    log_trace("--> Transfering frame: type: %d, daq ind:[%d]",iq_header->frame_type, iq_header->daq_block_index);
		    break;
                case 3: // Frame drop
                    break;
                default:
                    log_error("Failed to acquire free buffer");
                    exit_flag = 1;
            }
        }                
    }    
    error_code_log(exit_flag);
    log_info("Send terminate and wait..");
    send_ctr_terminate(output_sm_buff);
    sleep(3);    
    destory_sm_buffer(output_sm_buff);
    /* Free up buffers */     
    for(int m=0;m<ch_num;m++)
    {
        free((circ_buff_structs + m * sizeof(*circ_buff_structs))->iq_circ_buffer);       
    }
    log_info("Rebuffering block exited");
    return 0;
    
}


