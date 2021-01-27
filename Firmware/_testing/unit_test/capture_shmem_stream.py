"""
   HeIMDALL DAQ Firmware
   Description : Capture and record an IQ frame stream transfered via a shared memory interface
   Author      : Tamas Peto
   License     : GNU GPL V3
    
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.

   WARNING: Check the native size of the IQ header on the target device
"""

import logging
import sys
from os.path import join, dirname, realpath
from struct import pack, unpack
import numpy as np
from configparser import ConfigParser
import time
import threading

current_path      = dirname(realpath(__file__))
root_path         = dirname(dirname(current_path))
daq_core_path     = join(root_path, "_daq_core")

sys.path.insert(0, daq_core_path)
from iq_header import IQHeader
from shmemIface import inShmemIface

class IQFrameRecorder(threading.Thread):
    
    def __init__(self, shmem_name, out_fname, en_write):        
        threading.Thread.__init__(self)
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger(__name__)        
        
        self.fw_ctr_gen_frame = None
        self.bw_ctr_gen_frame = None              
        
        self.iq_header = IQHeader()
        
        # Open record output file
        self.output_file = open(out_fname,"wb")
        
        self.en_write = int(en_write)

        self.start_time = 0
        self.elapsed_time = 0
        # Open shared memory interface to capture the module output
        self.in_shmem_iface = inShmemIface(shmem_name)
        if not self.in_shmem_iface.init_ok:
            self.logger.critical("Shared memory initialization failed")
            self.in_shmem_iface.destory_sm_buffer()        
    
    def run(self):
        """
            Start the main processing loop
        """
        logging.info("Starting IQ frame capture on shared memory iface")
        while True:            
            #############################################
            #           OBTAIN NEW DATA BLOCK           #  
            #############################################
                        
            # Read and convert header            
            active_buff_index = self.in_shmem_iface.wait_buff_free()            
            if self.start_time == 0:
                self.start_time = time.time()
                
            if active_buff_index < 0 or active_buff_index > 1:
                logging.info("Terminating.., signal: {:d}".format(active_buff_index))                
                break;          
            
            buffer = self.in_shmem_iface.buffers[active_buff_index]
                       
            iq_header_bytes = buffer[0:1024].tobytes()
            iq_frame_bytes = bytearray()+iq_header_bytes
            
            self.iq_header.decode_header(iq_header_bytes)            
            if self.iq_header.check_sync_word():
                logging.error("IQ header sync word check failed, exiting..")
                break                        
            #self.iq_header.dump_header()            
            incoming_payload_size = self.iq_header.cpi_length*self.iq_header.active_ant_chs*2*int(self.iq_header.sample_bit_depth/8)                                                            
            if incoming_payload_size > 0:                
                iq_samples = buffer[1024:1024 + incoming_payload_size].view(dtype=np.float32)
                iq_frame_bytes +=  iq_samples.tobytes()     
            if self.en_write:
                self.output_file.write(iq_frame_bytes)            
            
            self.logger.info("IQ frame received. [{:d}]".format(self.iq_header.daq_block_index))
            self.in_shmem_iface.send_ctr_buff_ready(active_buff_index)
            #self.logger.info("Buff ready: {:d}".format(active_buff_index))
            
        self.elapsed_time = time.time()-self.start_time        
        self.in_shmem_iface.destory_sm_buffer()
        self.output_file.close()
        logging.info("Capture module exited")
        

if __name__ == '__main__':
    recorder = IQFrameRecorder(sys.argv[1],sys.argv[2],sys.argv[3])
    if recorder.in_shmem_iface.init_ok:
        recorder.start()
    else:
        exit()

    
    
