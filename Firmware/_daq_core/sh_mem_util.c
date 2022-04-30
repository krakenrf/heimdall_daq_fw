/*
 * 
 * Description :
 * Util functions to handle shared memory based IQ frame transfer between processing blocks
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
#include <fcntl.h> 
#include <sys/shm.h> 
#include <sys/stat.h> 
#include <sys/mman.h>

#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include "sh_mem_util.h"
#include "log.h"

#define CHK_SUCC(r, e)    if(r != 0)  {return e;}
#define CHK_ZERO(r, e)    if(r == 0)  {return e;}
#define CHK_READ(r, s ,e) if(r != s)  {return e;}
#define CHK_DATA_PIPE(fd, e) if(feof(fd)) {return e;}

unsigned char char_init_ready[1]={INIT_READY}; 
unsigned char char_a_buff_ready[1]={A_BUFF_READY}; 
unsigned char char_b_buff_ready[1]={B_BUFF_READY}; 
unsigned char char_terminate[1]={TERMINATE}; 

uint8_t signal;

void send_ctr_init_ready(struct shmem_transfer_struct* sm_buff)
{       
    fwrite(char_init_ready,1,1,sm_buff->fw_ctr_fifo);
	fflush(sm_buff->fw_ctr_fifo);
}
void send_ctr_terminate(struct shmem_transfer_struct* sm_buff)
{
    fwrite(char_terminate,1,1,sm_buff->fw_ctr_fifo);
	fflush(sm_buff->fw_ctr_fifo);
}
void send_ctr_buff_ready(struct shmem_transfer_struct* sm_buff, int active_buff_index)
{
    sm_buff->buffer_free[active_buff_index] = false;    
    if      (active_buff_index == 0){fwrite(char_a_buff_ready,1,1,sm_buff->fw_ctr_fifo);}
    else if (active_buff_index == 1){fwrite(char_b_buff_ready,1,1,sm_buff->fw_ctr_fifo);}
    fflush(sm_buff->fw_ctr_fifo); 

}
void send_ctr_buff_free(struct shmem_transfer_struct* sm_buff, int active_buff_index)
{  
    if      (active_buff_index == 0){fwrite(char_a_buff_ready,1,1,sm_buff->bw_ctr_fifo);}
    else if (active_buff_index == 1){fwrite(char_b_buff_ready,1,1,sm_buff->bw_ctr_fifo);}
    fflush(sm_buff->bw_ctr_fifo); 
}

int wait_ctr_init_ready(struct shmem_transfer_struct* sm_buff)
{    
    int read_size=fread(&signal, sizeof(signal), 1, sm_buff->fw_ctr_fifo);        
    CHK_READ(read_size, 1 ,-1)    
    if(signal == INIT_READY) {return 0;}
    else {return -2;}
}
int wait_buff_free(struct shmem_transfer_struct* sm_buff)
{
    if (sm_buff->buffer_free[0] == true)
        return 0;
    else if (sm_buff->buffer_free[1] == true)
        return 1;    
    else
    { 
        int read_size=fread(&signal, sizeof(signal), 1, sm_buff->bw_ctr_fifo);
        switch (read_size) 
        { 
            case 0:  // PIPE empty and errono set EAGAIN
                if (errno == EAGAIN) 
                { 
                	sm_buff->dropped_frame_cntr +=1;
                    if (INGORE_FRAME_DROP_WARNINGS==0)
                        log_warn("Dropping frame.. Total: [%d]",sm_buff->dropped_frame_cntr);
                    return 3;                        
                } 
                else 
                { 
                    log_error("Backward control FIFO read error");
                    return -1;
                }
                break;     
            case EOF:  // EOF, Pipe closed
                log_error("Unhandled bw fifo close");
                return -1;
                break;                    
            case 1:
                if(signal == A_BUFF_READY)
                {
                    sm_buff->buffer_free[0] = true;
                    return 0;
                }
                else if(signal == B_BUFF_READY)
                {
                    sm_buff->buffer_free[1] = true;
                    return 1;
                }
                else
                {
                    log_error("Unidentified control signal: %d", signal);
                    return -1;
                }        
            break;
        }
    }
    return -2;
}
int wait_buff_ready(struct shmem_transfer_struct* sm_buff)
{
    uint8_t signal;      
    CHK_DATA_PIPE(sm_buff->fw_ctr_fifo, -1);
    int read_size=fread(&signal, sizeof(signal), 1, sm_buff->fw_ctr_fifo);        
    CHK_READ(read_size, 1 ,-1)          
    if(signal == A_BUFF_READY){return 0;}
    else if(signal == B_BUFF_READY){return 1;}
    else if (signal == TERMINATE){return TERMINATE;}

    return -2;
}

int init_out_sm_buffer(struct shmem_transfer_struct* sm_buff) 
{
    /* Create the shared memory object */
    sm_buff->shm_fd[0] = shm_open(sm_buff->shared_memory_names[0], O_CREAT | O_RDWR, 0666); 
    CHK_ZERO(sm_buff->shm_fd[0], -1)
    sm_buff->shm_fd[1] = shm_open(sm_buff->shared_memory_names[1], O_CREAT | O_RDWR, 0666);     
    CHK_ZERO(sm_buff->shm_fd[1], -1)    
  
    /* Configure the size of the shared memory object */    
    int ret = ftruncate(sm_buff->shm_fd[0], sm_buff->shared_memory_size); 
    CHK_SUCC(ret, -2)
    ret = ftruncate(sm_buff->shm_fd[1], sm_buff->shared_memory_size); 
    CHK_SUCC(ret, -2)

    /* Memory map the shared memory object */    
    sm_buff->shm_ptr[0] = mmap(0, sm_buff->shared_memory_size, PROT_WRITE, MAP_SHARED, sm_buff->shm_fd[0], 0); 
    CHK_ZERO(sm_buff->shm_ptr[0], -3)
    sm_buff->shm_ptr[1] = mmap(0, sm_buff->shared_memory_size, PROT_WRITE, MAP_SHARED, sm_buff->shm_fd[1], 0); 
    CHK_ZERO(sm_buff->shm_ptr[1], -3)

    /* Open forward control FIFO*/
    sm_buff->fw_ctr_fifo = fopen(sm_buff->fw_ctr_fifo_name, "w");
    CHK_ZERO(sm_buff->fw_ctr_fifo, -4);

    /* Open backward control FIFO*/
    sm_buff->bw_ctr_fifo= fopen(sm_buff->bw_ctr_fifo_name, "r");
    if (sm_buff->drop_mode)
    {
         int ret= fcntl(fileno(sm_buff->bw_ctr_fifo), F_SETFL, fcntl(fileno(sm_buff->bw_ctr_fifo), F_GETFL) |  O_NONBLOCK);                               
        CHK_SUCC(ret, -4);
    }
    
    CHK_ZERO(sm_buff->bw_ctr_fifo, -5);

    sm_buff->buffer_free[0] = true;
    sm_buff->buffer_free[1] = true;
    
    sm_buff->dropped_frame_cntr = 0;
    
    send_ctr_init_ready(sm_buff);

    return 0;
}
int init_in_sm_buffer(struct shmem_transfer_struct* sm_buff) 
{
    /* Open forward control FIFO*/
    sm_buff->fw_ctr_fifo = fopen(sm_buff->fw_ctr_fifo_name, "r");
    CHK_ZERO(sm_buff->fw_ctr_fifo, -1)

    /* Open backward control FIFO*/
    sm_buff->bw_ctr_fifo= fopen(sm_buff->bw_ctr_fifo_name, "w");
    CHK_ZERO(sm_buff->bw_ctr_fifo, -2)

    /* Check init ready success on the generator side*/
    int ret = wait_ctr_init_ready(sm_buff);
    CHK_SUCC(ret, -3)

    /* Create the shared memory object */
    sm_buff->shm_fd[0] = shm_open(sm_buff->shared_memory_names[0], O_RDWR, 0666); 
    CHK_ZERO(sm_buff->shm_fd[0], -4)
    sm_buff->shm_fd[1] = shm_open(sm_buff->shared_memory_names[1], O_RDWR, 0666);     
    CHK_ZERO(sm_buff->shm_fd[1], -4)    
  
    /* Memory map the shared memory object */    
    sm_buff->shm_ptr[0] = mmap(0, sm_buff->shared_memory_size, PROT_READ, MAP_SHARED, sm_buff->shm_fd[0], 0); 
    CHK_ZERO(sm_buff->shm_ptr[0], -5)
    sm_buff->shm_ptr[1] = mmap(0, sm_buff->shared_memory_size, PROT_READ, MAP_SHARED, sm_buff->shm_fd[1], 0); 
    CHK_ZERO(sm_buff->shm_ptr[1], -5)
    
    sm_buff->dropped_frame_cntr = 0;
    
    return 0;
}

int destory_sm_buffer(struct shmem_transfer_struct* sm_buff)
{
    /* Unmap the shared memory object */    
    int ret = munmap(sm_buff->shm_ptr[0], sm_buff->shared_memory_size);
    CHK_SUCC(ret, -1)
    ret = munmap(sm_buff->shm_ptr[1], sm_buff->shared_memory_size);
    CHK_SUCC(ret, -1)
    
    if( sm_buff->io_type == 0)
    {
        /* Remove shared memory object */    
        ret = shm_unlink(sm_buff->shared_memory_names[0]);
        CHK_SUCC(ret, -2)
        ret = shm_unlink(sm_buff->shared_memory_names[1]);
        CHK_SUCC(ret, -2)
    }

    /* Close forward control FIFO*/
    fclose(sm_buff->fw_ctr_fifo);

    /* Close backward control FIFO*/
    fclose(sm_buff->bw_ctr_fifo);

    return 0;
}
