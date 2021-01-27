/*
 * 
 * Description :
 * Realize configurable sample delay for multichannel coherent receivers.
 * Input data read from the std. input is stored in a circular buffer. This 
 * data is writen to the std. output from the circular buffer with setting the
 * requested sample offsets.
 *
 * Project : HeIMDALL DAQ Firmware
 * License : GNU GPL V3
 * Author  : Tamas Peto
 *
 * Copyright (C) 2018-2020  Tamás Pető
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

/* Implementation note:
*  The internally used buffer size variable denotes the number of downloaded IQ samples 
*  Eg: When the buffer_size has a value of 2**18, then 2**18 IQ sample is actually downloaded per channel
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <semaphore.h>
#include "rtl_daq.h"
#include "log.h"
#include "ini.h"
#include "iq_header.h"

#define BUFFER_NO 2  // Buffer number
#define INI_FNAME "daq_chain_config.ini"
#define SYNC_CFN "_data_control/sync_control_fifo"
/*
 * This structure stores the configuration parameters, 
 * that are loaded from the ini file
 */ 
typedef struct
{
    int num_ch;
    int daq_buffer_size;
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
    if (MATCH("daq", "daq_buffer_size")) 
    {
        pconfig->daq_buffer_size = atoi(value);
    } 
    else if (MATCH("hw", "num_ch")) 
    {
        pconfig->num_ch = atoi(value);
    } 
    else if (MATCH("daq", "log_level")) 
    {
        pconfig->log_level = atoi(value);
    }
    else {
        return 0;  /* unknown section/name, error */
    }
    return 0;
}
// TODO: Replace semaphore with pthread mutex and signal cond 
static sem_t trigger_sem;
static int control_signal; /*0- nothing to do, 1- add delay values, 2 - reset delay values*/
static int exit_flag = 0;
static int *delays; 
pthread_t fifo_read_thread;    
static int ch_no, sample_size;
void * fifo_read_tf(void* arg) 
{
   /*   
    *  Control FIFO read thread function
    * 
    *  This thread function handles the external requests using an external FIFO file.
    *  Upon receipt of a command, this thread infroms the main thread on the requested operation.
    *  
    *  The valid (1 byte) commands are the followings:
    *       d: Set delay values 
    *       r: Reset delay values
    *       2: Gentle system halt request
    *  
    *  Return values:
    *  --------------
    *       NULL
    */   
    (void)arg; 
    uint8_t signal;
    int read_size;
    FILE * fd = fopen(SYNC_CFN, "r"); // FIFO descriptor    
    if(fd==0)    
    {
        log_fatal("FIFO open error");
        exit_flag = 1;
    }
    /* Main thread loop*/    
    while(!exit_flag){
        read_size=fread(&signal, sizeof(signal), 1, fd);
        CHK_CTR_READ(read_size,1);   
        /* System halt request*/
        if( (uint8_t) signal == 2)
        {            
            log_info("Signal 2: FIFO read thread exiting");
            exit_flag = 1;
            break;
        }
        /* Reset delay values*/
        else if( (char) signal == 'r')
        {
            log_info("Signal 'r': Reseting delay values");
            control_signal=2; // Set control_signal for updating delay values
            sem_wait(&trigger_sem);             
        }
        /* Update delay values*/
        else if( (char) signal == 'd')
        {
            log_info("Signal 'd': Updating delay values");
            read_size=fread(delays, sizeof(*delays), ch_no, fd);
            CHK_CTR_READ(read_size, ch_no);            
            for(int m=0; m < ch_no; m++)     
                if(abs(delays[m]) < sample_size)
                {
                    log_info("Channel %d, delay value %d",m,delays[m]);
                    delays[m] *=2; // I and Q !
                }
                else
                {
                    log_error("Delay value over buffer size: %d, Setting to zero",delays[m]);
                    delays[m] = 0;
                    log_warn("Channel %d, delay value %d",m,delays[m]);
                }
            control_signal=1; // Set control_signal for updating delay values
            sem_wait(&trigger_sem);          
        }
    }
    fclose(fd);
    return NULL;
}

int main(int argc, char* argv[])
{
    log_set_level(LOG_TRACE); // Defualt log level is changed when the ini file is read
    configuration config;
    
    /* Set parameters from the config file*/    
    if (ini_parse(INI_FNAME, handler, &config) < 0) {
        log_fatal("Configuration could not be loaded, exiting ..");
        return -1;
    }    
    sample_size = config.daq_buffer_size;
    ch_no = config.num_ch;
    log_set_level(config.log_level);          
    log_info("Config succesfully loaded from: %s",INI_FNAME);
    log_info("Channel number: %d", ch_no);
    log_info("Number of IQ samples per channel: %d", sample_size);
    
    delays = (int*) malloc(ch_no*sizeof(int));    
    int read_size; // Stores the read bytes from stdin
    int first_read = 0;
    int write_index = BUFFER_NO-1; // Buffer index 0..BUFFER_NO-1
    uint8_t *read_pointer;    
    struct iq_header_struct* iq_header = calloc(1, sizeof(struct iq_header_struct));    

    /* Initializing control thread */ 
    sem_init(&trigger_sem, 0, 0);  // Semaphore is unlocked
    control_signal = 0;
       
    pthread_create(&fifo_read_thread, NULL, fifo_read_tf, NULL);
   
    /* Initializing delay buffers */
    struct sync_buffer_struct* sync_buffers;    
    sync_buffers = malloc(ch_no*sizeof(*sync_buffers));
    for(int m=0;m<ch_no;m++)
    {
        sync_buffers[m].circ_buffer = malloc(BUFFER_NO*sample_size*2*sizeof(uint8_t)); // *2-> I and Q    
        sync_buffers[m].delay = sample_size;  
    }
    /*
     *
     * ---> Main processing loop <---
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
        //dump_iq_header(iq_header); // Uncomment to debug the header content
        CHK_FR_READ(read_size,1);   
        CHK_SYNC_WORD(check_sync_word(iq_header));
        log_trace("<-- Frame received: type: %d, daq ind:[%d]",iq_header->frame_type, iq_header->daq_block_index);

        /* Reading multichannel IQ data */
        write_index = (write_index+1)% BUFFER_NO;
        log_debug("Buffer write index %d", write_index);
        if (iq_header->cpi_length > 0)
        {
            for(int m=0;m<iq_header->active_ant_chs;m++)
            {
                read_size = fread(sync_buffers[m].circ_buffer +(sample_size*2*write_index), sizeof(uint8_t), (sample_size*2), stdin);                
                CHK_FR_READ(read_size, sample_size*2);                                                                  
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
                for(int m=0;m<ch_no;m++)
                {
                    memset(sync_buffers[m].circ_buffer, 0, BUFFER_NO*sample_size*2*sizeof(uint8_t));
                }      
                first_read  = 0;
                write_index = BUFFER_NO-1; // Buffer index 0..BUFFER_NO-1
            }
            break;
            case FRAME_TYPE_DATA:
            case FRAME_TYPE_CAL:
            {
                if(first_read < 2)
                    first_read++;
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
            /* Writing IQ header */
            iq_header->cpi_length = 0;
            fwrite(iq_header, sizeof(struct iq_header_struct), 1, stdout);
        }
        else if (first_read == 1)
        {
            /* Writing Dummy Frame */
            iq_header->frame_type = FRAME_TYPE_DUMMY;
            iq_header->cpi_length = 0;
            fwrite(iq_header, sizeof(struct iq_header_struct), 1, stdout);

        }
        //Data is writen only after the second frame has arrived
        else if (first_read == 2)
        {       
            /* Writing IQ header */            
            fwrite(iq_header, sizeof(struct iq_header_struct), 1, stdout);

            /* Writing Multichannel IQ data */
            for(int m=0; m<ch_no; m++)
            {   
                /* Write from delay pointer*/ 
                int delay = sync_buffers[m].delay;
                log_debug("first read %d, wr index: %d",first_read, write_index);
                if(write_index == 1)
                {
                    log_debug("Write in one chunk");
                    read_pointer = sync_buffers[m].circ_buffer + delay;
                    fwrite(read_pointer , sizeof(uint8_t), sample_size*2, stdout);
                    fflush(stdout);
                }
                else // Write index must be 0
                {
                    log_debug("Write in two chunk");
                    // Write first chunk
                    read_pointer = sync_buffers[m].circ_buffer + (delay+sample_size*2);
                    fwrite(read_pointer , sizeof(uint8_t), (sample_size*2-delay), stdout);                        
                    
                    //Write second chunk
                    read_pointer = sync_buffers[m].circ_buffer;
                    fwrite(read_pointer , sizeof(uint8_t), delay, stdout);
                    fflush(stdout);
                }                   
                log_debug("Channel: %d, Delay: %d [%ld]",m,delay,iq_header->daq_block_index);            
            } // End of multichannel data block read-write
            log_trace("--> Transfering frame: type: %d, daq ind:[%d]",iq_header->frame_type, iq_header->daq_block_index);
        }
        /*
         *----------------------
         *  External control
         *----------------------
        */        
        /* Update delay values*/
        if(control_signal == 1)
        {              
            for(int k=0; k<ch_no; k++)
            {
                sync_buffers[k].delay += delays[k];
                delays[k] = 0;
            }
            control_signal = 0;
            sem_post(&trigger_sem);
        }
        /* Reset delay values*/
        else if(control_signal == 2)
        {              
            for(int k=0; k<ch_no; k++)
            {
                sync_buffers[k].delay = sample_size; 
            }
            control_signal = 0;
            sem_post(&trigger_sem);
        }        
    }
    error_code_log(exit_flag);
    /* Free up buffers */ 
    for(int m=0;m<ch_no;m++)
    {
        free(sync_buffers[m].circ_buffer);       
    }
    sem_destroy(&trigger_sem);   
    log_info("Sync module exited");
    return 0;    
}


