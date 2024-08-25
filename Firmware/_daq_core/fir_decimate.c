/*
 *
 * Description
 * Implements FIR filter based decimator using the KFR library.
 *
 * Project : HeIMDALL DAQ Firmware
 * License : GNU GPL V3
 * Author  : Tamás Peto
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

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include "log.h"
#include "ini.h"
#include "iq_header.h"
#include "sh_mem_util.h"
#include "rtl_daq.h"

#include <kfr/capi.h>

#define DC 127.5
#define INI_FNAME "daq_chain_config.ini"
#define FIR_COEFF "_data_control/fir_coeffs.txt"
#define FATAL_ERR(l) \
    log_fatal(l);    \
    return -1;
#define CHK_MALLOC(m)                          \
    if (m == NULL)                             \
    {                                          \
        log_fatal("Malloc failed, exiting.."); \
        return -1;                             \
    }
/*
 * This structure stores the configuration parameters,
 * that are loaded from the ini file
 */
typedef struct
{
    int num_ch;
    int cpi_size;
    int cal_size;
    int decimation_ratio;
    int en_filter_reset;
    int tap_size;
    int log_level;
} configuration;

/*
 * Ini configuration parser callback function
 */
static int handler(void *conf_struct, const char *section, const char *name,
                   const char *value)
{
    configuration *pconfig = (configuration *)conf_struct;
#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    if (MATCH("hw", "num_ch"))
    {
        pconfig->num_ch = atoi(value);
    }
    else if (MATCH("pre_processing", "cpi_size"))
    {
        pconfig->cpi_size = atoi(value);
    }
    else if (MATCH("calibration", "corr_size"))
    {
        pconfig->cal_size = atoi(value);
    }
    else if (MATCH("pre_processing", "decimation_ratio"))
    {
        pconfig->decimation_ratio = atoi(value);
    }
    else if (MATCH("pre_processing", "en_filter_reset"))
    {
        pconfig->en_filter_reset = atoi(value);
    }
    else if (MATCH("pre_processing", "fir_tap_size"))
    {
        pconfig->tap_size = atoi(value);
    }
    else if (MATCH("daq", "log_level"))
    {
        pconfig->log_level = atoi(value);
    }
    else
    {
        return 0; /* unknown section/name, error */
    }
    return 0;
}
int main(int argc, char **argv)
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
    bool filter_reset;
    int ch_no, dec;
    int exit_flag = 0;
    int active_buff_ind = 0, active_buff_ind_in = 0;
    bool drop_mode = true;

    struct iq_header_struct *iq_header;
    uint8_t *input_data_buffer;
    /* Set drop mode from the command prompt*/
    if (argc == 2)
    {
        drop_mode = atoi(argv[1]);
    }

    /* Set parameters from the config file*/
    if (ini_parse(INI_FNAME, handler, &config) < 0)
    {
        FATAL_ERR("Configuration could not be loaded, exiting ..")
    }

    ch_no = config.num_ch;
    dec = config.decimation_ratio;
    filter_reset = (bool)config.en_filter_reset;
    log_set_level(config.log_level);
    log_info("Config succesfully loaded from %s", INI_FNAME);
    log_info("Channel number: %d", ch_no);
    log_info("Decimation ratio: %d", dec);
    log_info("CPI size: %d", config.cpi_size);
    log_info("Calibration sample size : %d", config.cal_size);

    /*
     *-------------------------------------
     *  Allocation and initialization
     *-------------------------------------
     */

    /* Initializing input shared memory interface */
    struct shmem_transfer_struct *input_sm_buff = calloc(1, sizeof(struct shmem_transfer_struct));
    if ((config.cpi_size * dec) >= config.cal_size)
    {
        input_sm_buff->shared_memory_size = config.cpi_size * config.num_ch * dec * 4 * 2 + IQ_HEADER_LENGTH;
    }
    else
    {
        input_sm_buff->shared_memory_size = config.cal_size * config.num_ch * 4 * 2 + IQ_HEADER_LENGTH;
    }
    input_sm_buff->io_type = 1; // Input type

    strcpy(input_sm_buff->shared_memory_names[0], DECIMATOR_IN_SM_NAME_A);
    strcpy(input_sm_buff->shared_memory_names[1], DECIMATOR_IN_SM_NAME_B);
    strcpy(input_sm_buff->fw_ctr_fifo_name, DECIMATOR_IN_FW_FIFO);
    strcpy(input_sm_buff->bw_ctr_fifo_name, DECIMATOR_IN_BW_FIFO);

    int succ = init_in_sm_buffer(input_sm_buff);
    if (succ != 0)
    {
        FATAL_ERR("Input shared memory initialization failed")
    }
    else
    {
        log_info("Input shared memory interface successfully initialized");
    }

    /* Initializing output shared memory interface */
    struct shmem_transfer_struct *output_sm_buff = calloc(1, sizeof(struct shmem_transfer_struct));
    output_sm_buff->shared_memory_size = MAX_IQFRAME_PAYLOAD_SIZE * ch_no * 4 * 2 + IQ_HEADER_LENGTH;
    output_sm_buff->io_type = 0; // Output type
    output_sm_buff->drop_mode = drop_mode;
    strcpy(output_sm_buff->shared_memory_names[0], DECIMATOR_OUT_SM_NAME_A);
    strcpy(output_sm_buff->shared_memory_names[1], DECIMATOR_OUT_SM_NAME_B);
    strcpy(output_sm_buff->fw_ctr_fifo_name, DECIMATOR_OUT_FW_FIFO);
    strcpy(output_sm_buff->bw_ctr_fifo_name, DECIMATOR_OUT_BW_FIFO);

    succ = init_out_sm_buffer(output_sm_buff);
    if (succ != 0)
    {
        FATAL_ERR("Shared memory initialization failed")
    }
    else
    {
        log_info("Output shared memory interface successfully initialized");
    }

    size_t tap_size = config.tap_size;
    /* Allocating FIR filter data buffers */

    kfr_f32 *fir_input_buffer_i = kfr_allocate(config.cpi_size * dec * sizeof(kfr_f32));
    kfr_f32 *fir_input_buffer_q = kfr_allocate(config.cpi_size * dec * sizeof(kfr_f32));

    kfr_f32 *fir_output_buffer_i = kfr_allocate(config.cpi_size * dec * sizeof(kfr_f32));
    kfr_f32 *fir_output_buffer_q = kfr_allocate(config.cpi_size * dec * sizeof(kfr_f32));

    kfr_f32 *fir_coeffs = kfr_allocate(tap_size * sizeof(kfr_f32));

    /* Initialize Filter taps */
    FILE *fir_coeff_fd = fopen(FIR_COEFF, "r");
    if (fir_coeff_fd == NULL)
    {
        FATAL_ERR("Failed to open FIR coefficient file")
    }
    int k = 0;
    while (fscanf(fir_coeff_fd, "%f", &fir_coeffs[k++]) != EOF)
        ;
    if (k - 1 == tap_size)
    {
        log_info("FIR filter coefficienrs are initialized, tap size: %d", k - 1);
    }
    else
    {
        FATAL_ERR("FIR filter coefficients initialization failed")
    }

    /* Initializing FIR filter */
    KFR_FILTER_F32 *fir_filter_plan = kfr_filter_create_fir_plan_f32(fir_coeffs, tap_size);
    uint64_t cpi_index = -1;
    void *frame_ptr;
    /* Main Processing loop*/
    while (!exit_flag)
    {

        // Acquire data buffer on the shared memory interface
        active_buff_ind_in = wait_buff_ready(input_sm_buff);
        if (active_buff_ind_in < 0)
        {
            exit_flag = 1;
            break;
        }
        if (active_buff_ind_in == TERMINATE)
        {
            exit_flag = TERMINATE;
            break;
        }
        iq_header = (struct iq_header_struct *)input_sm_buff->shm_ptr[active_buff_ind_in];
        input_data_buffer = ((uint8_t *)input_sm_buff->shm_ptr[active_buff_ind_in]) + IQ_HEADER_LENGTH / sizeof(uint8_t);
        CHK_SYNC_WORD(check_sync_word(iq_header));

        cpi_index++;

        /*Acquire buffer from the sink block*/
        active_buff_ind = wait_buff_free(output_sm_buff);
        switch (active_buff_ind)
        {
        case 0:
        case 1:
            log_trace("--> Frame received: type: %d, daq ind:[%d]", iq_header->frame_type, iq_header->daq_block_index);
            frame_ptr = output_sm_buff->shm_ptr[active_buff_ind];
            float *output_data_buffer = ((float *)output_sm_buff->shm_ptr[active_buff_ind]) + IQ_HEADER_LENGTH / sizeof(float);
            /* Place IQ header into the output buffer*/
            memcpy(frame_ptr, iq_header, 1024);

            /* Update header fields */
            iq_header = (struct iq_header_struct *)frame_ptr;
            iq_header->data_type = 3;         // Data type is decimated IQ
            iq_header->sample_bit_depth = 32; // Complex float 32
            iq_header->cpi_index = cpi_index;

            if (iq_header->frame_type == FRAME_TYPE_DATA && dec > 1)
            {
                iq_header->sampling_freq = iq_header->adc_sampling_freq / (uint64_t)dec;
                iq_header->cpi_length = (uint32_t)iq_header->cpi_length / dec;

                /* Perform filtering on data type frames*/
                if (iq_header->cpi_length > 0)
                {
                    if (filter_reset)
                        for (int ch_index = 0; ch_index < iq_header->active_ant_chs; ch_index++)
                        {
                            // For downsampling
                            int out_sample_index = 0;
                            int dec_index = 0;
                            // De-interleaving input data
                            for (int sample_index = 0; sample_index < iq_header->cpi_length * dec; sample_index++)
                            {
                                fir_input_buffer_i[sample_index] = (input_data_buffer[2 * sample_index] - DC) / DC;     // I
                                fir_input_buffer_q[sample_index] = (input_data_buffer[2 * sample_index + 1] - DC) / DC; // Q
                            }
                            // Perform filtering
                            kfr_filter_process_f32(fir_filter_plan, fir_output_buffer_i, fir_input_buffer_i, iq_header->cpi_length * dec);
                            kfr_filter_process_f32(fir_filter_plan, fir_output_buffer_q, fir_input_buffer_q, iq_header->cpi_length * dec);

                            // Downsample and re-interleave output data
                            for (int sample_index = 0; sample_index < iq_header->cpi_length * dec; sample_index++)
                            {
                                dec_index = (dec_index + 1) % dec; // when zero the sample is forwarded
                                if (dec_index == 0)
                                {
                                    output_data_buffer[out_sample_index] = fir_output_buffer_i[sample_index];
                                    output_data_buffer[out_sample_index + 1] = fir_output_buffer_q[sample_index];
                                    out_sample_index += 2;
                                }
                            }
                            input_data_buffer += 2 * iq_header->cpi_length * dec;
                            output_data_buffer += 2 * iq_header->cpi_length;
                        }
                }
            }
            else if (iq_header->frame_type == FRAME_TYPE_CAL || dec == 1)
            {
                iq_header->sampling_freq = iq_header->adc_sampling_freq;
                iq_header->cpi_length = (uint32_t)iq_header->cpi_length;

                /* Convert cint8 to cfloat32 without filtering and decimation on cal type frames*/
                for (int sample_index = 0; sample_index < iq_header->cpi_length * iq_header->active_ant_chs; sample_index++)
                {
                    output_data_buffer[2 * sample_index] = (float)(input_data_buffer[2 * sample_index] - DC) / DC;         // I
                    output_data_buffer[2 * sample_index + 1] = (float)(input_data_buffer[2 * sample_index + 1] - DC) / DC; // Q
                }
            }
            log_trace("<--Transfering frame type: %d, daq ind:[%d]", iq_header->frame_type, iq_header->daq_block_index);
            send_ctr_buff_ready(output_sm_buff, active_buff_ind);
            break;
        case 3:
            /* Frame drop*/
            break;
        default:
            log_error("Failed to acquire free buffer");
            exit_flag = 1;
        }
        send_ctr_buff_free(input_sm_buff, active_buff_ind_in);
    } // End of the main processing loop
    error_code_log(exit_flag);
    send_ctr_terminate(output_sm_buff);
    sleep(3);
    destory_sm_buffer(output_sm_buff);
    destory_sm_buffer(input_sm_buff);
    free(fir_input_buffer_i);
    free(fir_input_buffer_q);
    free(fir_output_buffer_i);
    free(fir_output_buffer_q);
    log_info("Decimator exited");
    return 0;
}
