"""
	Description :
	Ramp signal generator for unit testing 

	Project : HeIMDALL DAQ Firmware
	License : GNU GPL V3
	Author  : Tamas Peto

	Command line arguments:
		-t : Frame type (default:DATA)
		-b : Number of frames to send (default:10)
		-n : Number of samples per channel in the payload
 
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
import logging
import threading

# Import external modules
currentPath = os.path.dirname(os.path.realpath(__file__))
rootPath = os.path.dirname(os.path.dirname(currentPath))
sys.path.insert(0, os.path.join(rootPath, "_daq_core"))
from iq_header import IQHeader
from shmemIface import outShmemIface

class rampFrameGenator(threading.Thread):

	def __init__(self, frame_type=0, 
              		blocks = 10,
					N_daq=2**18,
					data_type='CINT8',
					shmem_name=''):
		threading.Thread.__init__(self)
		logging.basicConfig(level=logging.INFO)
		self.logger = logging.getLogger(__name__)
		
		self.N_daq      = N_daq
		self.M          = 4 
		self.blocks     = blocks
		self.frame_type = frame_type
		self.data_type  = data_type
		self.shmem_name = shmem_name 

		# Synchronization related parameters
		self.delays     = [0, 0, 0, 0]
		self.ramp_max   = 29

		# Initialize IQ header
		self.iq_header = IQHeader()
		self.iq_header.sync_word            = IQHeader.SYNC_WORD
		self.iq_header.frame_type           = frame_type
		self.iq_header.active_ant_chs       = self.M      
		self.iq_header.daq_block_index      = 0   
		self.iq_header.cpi_index            = 0

		if (frame_type == IQHeader.FRAME_TYPE_DUMMY) | (frame_type == IQHeader.FRAME_TYPE_TRIGW):
			self.iq_header.cpi_length       = 0
		elif (frame_type == IQHeader.FRAME_TYPE_DATA) | (frame_type == IQHeader.FRAME_TYPE_CAL):
			self.iq_header.cpi_length       = N_daq

		if data_type == "CINT8":
			self.iq_header.sample_bit_depth  = 8
		elif data_type == "CF32":
			self.iq_header.sample_bit_depth  = 32
		else:
			self.iq_header.sample_bit_depth  = 0
   
		# Uninitialzed header fields are all zero!
		self.logger.info("Frame type: {:d}".format(self.frame_type))
		self.logger.info("Number of frames: {:d}".format(self.blocks))
		self.logger.info("Number of samples per channel: {:d}".format(self.N_daq))
		self.logger.info("Frame data type: {:s}".format(data_type))
		if shmem_name != "":
			self.logger.info("Frames will be transfered on a shared memory interface")
			self.logger.info("Shared memory interface name: {:s}".format(shmem_name))

	def run(self):
		if self.shmem_name != "":
			self.out_shmem_iface = outShmemIface(self.shmem_name,
										int(1024+self.N_daq*2*self.M*(self.iq_header.sample_bit_depth/8)),
										drop_mode = False)
			if not self.out_shmem_iface.init_ok:
				self.logger.critical("Shared memory initialization failed, exiting..")
				self.out_shmem_iface.destory_sm_buffer()				
			else:
				self.logger.info("Shared memory interface succesfully initialized")

		self.logger.info("Starting test frame generation")
    
		####################################
		#            Simulation
		####################################

		# Allocation
		sig_out_ch1 = np.zeros((self.N_daq*2), dtype=np.uint8) # This array stores the samples that are written to output
		sig_out_chm = np.zeros((self.M, self.N_daq*2), dtype=np.uint8)
  
		raw_sig_ramp = np.zeros((self.N_daq+max(self.delays)), dtype = np.uint8)
		time_index = 0
		for b in range(self.blocks):               
			self.logger.info("Writing block: {:d}".format(b))        
			self.iq_header.daq_block_index = b
			
			payload_byte_array = bytearray()
			for m in range(self.M):
				raw_sig_m = np.arange(time_index + self.delays[m], time_index + self.delays[m]+self.N_daq,1, dtype=np.uint32)%self.ramp_max
				sig_out_ch1[0::2] = raw_sig_m[:]+2*m*self.ramp_max
				sig_out_ch1[1::2] = raw_sig_m[:]+(2*m+1)*self.ramp_max
				payload_byte_array += pack('B'*self.N_daq*2, *sig_out_ch1)
				sig_out_chm[m,:]  = sig_out_ch1[:]
			time_index+=self.N_daq    
			#######################################
			#           SEND DATA FRAME
			####################################### 
			if self.shmem_name != "":
				active_buffer_index = self.out_shmem_iface.wait_buff_free()
				logging.info("Buffer free: {:d}".format(active_buffer_index))
				if active_buffer_index !=3:
					# Get the shared memory buffer
					iq_frame_buffer_out = (self.out_shmem_iface.buffers[active_buffer_index]).view(dtype=np.uint8)

					# Get the IQ sample array from the buffer
					iq_samples_out = iq_frame_buffer_out[1024:1024+self.iq_header.cpi_length*2*self.iq_header.active_ant_chs]\
							.reshape(self.iq_header.active_ant_chs, self.iq_header.cpi_length*2)
					
					(self.out_shmem_iface.buffers[active_buffer_index])[0:1024] = np.frombuffer(self.iq_header.encode_header(), dtype=np.uint8)
					for m in range(self.M):
						iq_samples_out[m,:] = sig_out_chm[m,:]
					self.out_shmem_iface.send_ctr_buff_ready(active_buffer_index)
			else:				
				#self.iq_header.dump_header()
				sys.stdout.buffer.write(self.iq_header.encode_header()) # Write the IQ header
				sys.stdout.buffer.write(payload_byte_array) # Write IQ data
		if self.shmem_name != "":
			self.out_shmem_iface.send_ctr_terminate()
			time.sleep(2)
			self.out_shmem_iface.destory_sm_buffer()
			self.logger.info("Total dropped frames: {:d}".format(self.out_shmem_iface.dropped_frame_cntr))
		self.logger.info("Standard frame generator exited")
     
    
if __name__ == "__main__":
	logging.basicConfig(level=logging.INFO)
	N_daq      = 2**18 
	M          = 4 
	blocks     = 10
	frame_type = IQHeader.FRAME_TYPE_DATA
	data_type  = "CINT8" 
	shmem_name = "" 
	# Parse command line arguments
	opts, args = getopt.getopt(sys.argv[1:],"t:b:n:d:m:")
	for opt, arg in opts:
		if opt == '-t':
			frame_type = int(arg)
		elif opt == '-b':
			blocks = int(arg)
		elif opt == '-n':
			N_daq = int(arg)
		elif opt == '-d':
			data_type = arg
		elif opt == '-m':
			shmem_name = arg
	generator = rampFrameGenator(frame_type, blocks, N_daq, data_type, shmem_name)
	generator.start()