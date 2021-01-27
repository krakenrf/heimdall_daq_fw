"""
	Description :
	Signal generator for testing the squelch module

	Project : HeIMDALL DAQ Firmware
	License : GNU GPL V3
	Author  : Tamas Peto

	Command line arguments:
 
	Copyright (C) 2018-2020  Tamás Pető
	
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

"""
import os
import sys
import getopt
import time
import numpy as np
from struct import pack, unpack
import threading
import logging

# Import external modules
current_path      = os.path.dirname(os.path.realpath(__file__))
root_path         = os.path.dirname(os.path.dirname(current_path))
test_logs_path    = os.path.join(root_path, "_testing", "test_logs")
sys.path.insert(0, os.path.join(root_path, "_daq_core"))
from iq_header import IQHeader
from shmemIface import outShmemIface

from os.path import join
from plotly import graph_objects as go

class burstFrameGenator(threading.Thread):

    def __init__(self, test_case, shmem_name):
        threading.Thread.__init__(self)        
        self.logger = logging.getLogger(__name__)

        self.init_ok    = True
        self.M          = 4 
        self.frame_type = IQHeader.FRAME_TYPE_DATA        
        self.test_case = test_case
        self.shmem_name = shmem_name

        if self.test_case == '3_5':
            self.blocks      = 13
            self.pulse_start = 5
            self.pulse_end   = 7
            self.N_cpi       = 2**18
        elif self.test_case == '3_10':
            self.blocks      = 5
            self.N_cpi       = 256
            self.peak_1_pos  = 100
            self.peak_2_pos  = 130
            self.burst_1_len = 211
            self.burst_2_len = 197
        else:
            self.logger.fatal('Unidentified test case: {:s}'.format(test_case)) 
            self.init_ok = False           

        self.iq_header = IQHeader()
        self.iq_header.sync_word            = IQHeader.SYNC_WORD
        self.iq_header.frame_type           = self.frame_type
        self.iq_header.active_ant_chs       = self.M
        self.iq_header.daq_block_index      = 0   
        self.iq_header.cpi_index            = 0   
        self.iq_header.cpi_length           = self.N_cpi
        self.iq_header.sample_bit_depth     = 32
        # Uninitialzed header fields are all zero!

        self.logger.info("Frame type: {:d}".format(self.frame_type))
        self.logger.info("Number of frames: {:d}".format(self.blocks))
        self.logger.info("Number of samples per channel: {:d}".format(self.N_cpi))
        self.logger.info('Test case: {:s}'.format(self.test_case))
        self.logger.info("Shared memory interface name: {:s}".format(shmem_name))


    def run(self):            
        self.out_shmem_iface = outShmemIface(self.shmem_name,
                                    int(1024+self.N_cpi*2*self.M*(self.iq_header.sample_bit_depth/8)),
                                    drop_mode = False)
        if not self.out_shmem_iface.init_ok:
            self.logger.critical("Shared memory initialization failed, exiting..")
            self.out_shmem_iface.destory_sm_buffer()				
        else:
            self.logger.info("Shared memory interface succesfully initialized")

        self.logger.info("Starting test frame generation")
        
        fig = go.Figure()
        ####################################
        #            Simulation
        ####################################

        # Allocation
        sig_out_ch1 = np.zeros((self.N_cpi*2), dtype=np.uint8) # This array stores the samples that are written to output
        sig_out_chm = np.zeros((self.M, self.N_cpi*2), dtype=np.uint8)
        time_index = 0
        for b in range(self.blocks):               
            self.logger.info("Writing block: {:d}".format(b))        
            self.iq_header.daq_block_index = b
            self.iq_header.cpi_index = b
            ramp_max = 0
            
            payload_byte_array = bytearray()
            for m in range(self.M):
                #------------------
                #  Test case 3_5
                #------------------
                if self.test_case == '3_5':
                    if b < self.pulse_start:
                        raw_sig_m = np.zeros((self.N_cpi), dtype=np.uint8)
                    # -- > Pulse < --
                    elif b >=self.pulse_start and b <= self.pulse_end:                 
                        raw_sig_m = np.ones((self.N_cpi), dtype=np.uint8)
                    # -- > Pulse < --
                    elif b > self.pulse_end:
                        raw_sig_m = np.zeros((self.N_cpi), dtype=np.uint8)
                #------------------
                #  Test case 3_10
                #------------------                
                elif self.test_case == '3_10':
                    raw_sig_m = np.ones(self.N_cpi, dtype=np.uint8)*-0.1
                    # -- > Block 0  < --
                    if b == 1:
                        ramp_max = 29
                        if m==0:
                            raw_sig_m[self.peak_1_pos] = 1
                        raw_sig_m[self.peak_1_pos+1:self.N_cpi] = np.arange(0, self.N_cpi-self.peak_1_pos-1,1, dtype=np.uint32)%ramp_max
                    # -- > Block 1  < --
                    elif b == 2:
                        ramp_max = 29
                        burst_1_remain = self.burst_1_len -(self.N_cpi-self.peak_1_pos-1)                    
                        raw_sig_m[0:burst_1_remain] = np.arange(self.burst_1_len-burst_1_remain, self.burst_1_len,1, dtype=np.uint32)%ramp_max                    
                        if m==0:
                            raw_sig_m[self.peak_2_pos] = 1
                        raw_sig_m[self.peak_2_pos+1:self.N_cpi] = np.arange(0, self.N_cpi-self.peak_2_pos-1,1, dtype=np.uint32)%ramp_max
                    # -- > Block 2  < --
                    elif b == 3:
                        ramp_max = 29                                                
                        burst_2_remain = self.burst_2_len - (self.N_cpi-self.peak_2_pos-1)                    
                        raw_sig_m[0:burst_2_remain] = np.arange(self.burst_2_len-burst_2_remain, self.burst_2_len,1, dtype=np.uint32)%ramp_max                    
                
                mask = (raw_sig_m[:] >= 0).astype(int)
                sig_out_ch1[0::2] = raw_sig_m[:] + mask*2*m*ramp_max
                sig_out_ch1[1::2] = raw_sig_m[:] + mask*(2*m+1)*ramp_max
                sig_out_chm[m,:]  = sig_out_ch1[:]

            #######################################
            #           SEND DATA FRAME
            ####################################### 
            fig.add_trace(go.Scatter(x=np.arange(self.N_cpi),y=sig_out_chm[0,0::2], name = "Block {:d}".format(b), line=dict(width=2, dash='solid')))
            
            
            active_buffer_index = self.out_shmem_iface.wait_buff_free()
            self.logger.info("Buffer free: {:d}".format(active_buffer_index))
            if active_buffer_index !=3:
                # Get the shared memory buffer
                iq_frame_buffer_out = (self.out_shmem_iface.buffers[active_buffer_index]).view(dtype=np.uint8)
                
                # Get the IQ sample array from the buffer
                iq_samples_out = (iq_frame_buffer_out[1024:1024 + self.iq_header.cpi_length*2*self.iq_header.sample_bit_depth//8*self.iq_header.active_ant_chs].view(dtype=np.complex64))\
                                .reshape(self.iq_header.active_ant_chs, self.iq_header.cpi_length)
                
                (self.out_shmem_iface.buffers[active_buffer_index])[0:1024] = np.frombuffer(self.iq_header.encode_header(), dtype=np.uint8)
                for m in range(self.iq_header.active_ant_chs):
                    iq_samples_out[m,:].real = sig_out_chm[m,0::2]
                    iq_samples_out[m,:].imag = sig_out_chm[m,1::2]
                self.out_shmem_iface.send_ctr_buff_ready(active_buffer_index)                

        self.out_shmem_iface.send_ctr_terminate()
        time.sleep(2)
        self.out_shmem_iface.destory_sm_buffer()
        self.logger.info("Total dropped frames: {:d}".format(self.out_shmem_iface.dropped_frame_cntr))
        self.logger.info("Burst frame generator exited")
        
        fig.write_html(join(test_logs_path,'TestCase-3_10.html'))


if __name__ == "__main__":
    test_case = "3_5"
    # Parse command line arguments
    opts, args = getopt.getopt(sys.argv[1:],"c:")
    for opt, arg in opts:
        if opt == '-c':
            test_case = arg

    generator = burstFrameGenator(test_case)
    if generator.init_ok:
        generator.start()

