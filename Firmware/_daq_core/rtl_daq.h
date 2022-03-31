/*
 * 
 * Description :
 * Various descriptor structures for the DAQ chain 
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

#include <pthread.h>
#include <rtl-sdr.h>
#include <stdint.h>
#include "log.h"


#define ERR_IQFRAME_WRITE     10 
#define ERR_IQFRAME_READ      11
#define ERR_CMD_READ          12
#define ERR_IQFRAME_SYC_WORD  13
#define ERR_DATA_PIPE_CLOSE   14
#define ERR_CTR_THREAD_READ   15 

#define CHK_SYNC_WORD(r)   if(r != 0)     {exit_flag = ERR_IQFRAME_SYC_WORD; break;}
#define CHK_FR_WRITE(r, e) if(r != e)     {exit_flag = ERR_IQFRAME_WRITE;    break;}
#define CHK_FR_READ(r, e)  if(r != e)     {exit_flag = ERR_IQFRAME_READ;     break;}
#define CHK_CMD_READ(r, e) if(r != e)     {exit_flag = ERR_CMD_READ;         break;}
#define CHK_DATA_PIPE(fd)  if(feof(fd))   {exit_flag = ERR_DATA_PIPE_CLOSE;  break;}
#define CHK_CTR_READ(r, e) if(r != e)     {exit_flag = ERR_CTR_THREAD_READ;}


#define MAX_IQFRAME_PAYLOAD_SIZE 8388608 // 2^23[sample] per channel
//Should be greather than the cpi_size in the daq_chain_config.ini
void error_code_log(int exit_flag)
/*
 * Dump out error codes
 * 
 */
{
    switch (exit_flag)
    {
    case ERR_IQFRAME_WRITE:
        log_error("IQ frame sending failed");
        break;
    case ERR_IQFRAME_READ:
        log_fatal("IQ header read error ");
        break;
    case ERR_IQFRAME_SYC_WORD:
        log_fatal("IQ frame sync word check failed");
        break;   
    case ERR_DATA_PIPE_CLOSE:
        log_fatal("Unexpected data pipe close");
        break;
    case ERR_CMD_READ:
        log_fatal("Command read error");
        break;
    default:
        break;
    }

}

// HeIMDALL DAQ inter-modul message structure
struct hdaq_im_msg_struct { 
    // Total length: 128 byte
    uint8_t source_module_identifier;
    char command_identifier;
    uint8_t parameters[126];
};

struct rtl_rec_struct {
    int dev_ind, gain;
    rtlsdr_dev_t *dev;
    uint8_t *buffer;   
    unsigned long long buff_ind;
    pthread_t async_read_thread;        
    uint32_t center_freq, sample_rate;
};
struct sync_buffer_struct { // Each channel has a circular buffer struct
	uint32_t delay;
	uint8_t *circ_buffer;  // Circular buffer	
};

struct circ_buffer_struct { 
	uint8_t *iq_circ_buffer;  // Stores 8 bit int values (Complex int 8 format)	
};




