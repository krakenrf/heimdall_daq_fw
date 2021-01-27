"""
	Description :
	Standard frame generator for unit testing 

	Project : HeIMDALL DAQ Firmware
	License : GNU GPL V3
	Author  : Tamas Peto

    Command line arguments:
		-t : Frame type (default:DATA)
		-b : Number of frames to send (default:10)
		-n : Number of samples per channel in the payload
		-d : Data type "CINT16"/"CF64"(default:CINT16)
		-m : Shared memory name (if not specified, data is sent on std out)
  	
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

# Import IQ header module
currentPath = os.path.dirname(os.path.realpath(__file__))
rootPath = os.path.dirname(os.path.dirname(currentPath))
sys.path.insert(0, os.path.join(rootPath, "_daq_core"))
from iq_header import IQHeader
from shmemIface import outShmemIface

class stdFrameGenator(threading.Thread):

	def __init__(self, frame_type=0, 
              		blocks = 10,
					N_daq=2**18,
					data_type='CINT8',
					shmem_name=''):
		threading.Thread.__init__(self)		
		self.logger = logging.getLogger(__name__)
  
		self.N_daq      = N_daq
		self.M          = 4 
		self.blocks     = blocks
		self.frame_type = frame_type
		self.data_type  = data_type
		self.shmem_name = shmem_name 
    
		self.logger.info("Frame type: {:d}".format(frame_type))
		self.logger.info("Number of frames: {:d}".format(blocks))
		self.logger.info("Number of sample per channel: {:d}".format(N_daq))
		self.logger.info("Frame data type: {:s}".format(data_type))
		if shmem_name != "":
			self.logger.info("Frames will be transfered on a shared memory interface")
			self.logger.info("Shared memory interface name: {:s}".format(shmem_name))

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
			self.logger.critical("Unidentified data type: {:s}".format(data_type))
			self.iq_header.sample_bit_depth  = 0
   
		# Uninitialzed header fields are all zero!

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
		
		####################################
		#            Simulation
		####################################
		
		self.logger.info("Starting test frame generation")
		# Allocation
		if self.data_type == "CINT8":
				signal = np.zeros((self.N_daq*2), dtype=np.uint8) # This array stores the samples that are written to output
		elif self.data_type == "CF32":
				signal = np.zeros((self.N_daq*2), dtype=np.float32) # This array stores the samples that are written to output

		payload_byte_array = bytearray()
		for m in range(self.M):
			if self.data_type == "CINT8":
				payload_byte_array += pack('B'*self.N_daq*2, *signal)
			elif self.data_type == "CF32":
				payload_byte_array += pack('f'*self.N_daq*2, *signal)
		try:
			for b in range(self.blocks):               
				self.logger.info("Writing block: {:d}".format(b))        
				self.iq_header.daq_block_index = b
				self.iq_header.cpi_index = b
				#######################################
				#           SEND DATA FRAME
				#######################################        
				if self.shmem_name != "":
					active_buffer_index = self.out_shmem_iface.wait_buff_free()
					self.logger.info("Buffer free: {:d}".format(active_buffer_index))
					if active_buffer_index !=3:
						(self.out_shmem_iface.buffers[active_buffer_index])[0:1024] = np.frombuffer(self.iq_header.encode_header(), dtype=np.uint8)
						self.out_shmem_iface.send_ctr_buff_ready(active_buffer_index)
				else:
					sys.stdout.buffer.write(self.iq_header.encode_header()) # Write the IQ header
					if (self.frame_type == IQHeader.FRAME_TYPE_DATA)  | (self.frame_type == IQHeader.FRAME_TYPE_CAL):
						sys.stdout.buffer.write(payload_byte_array) # Write IQ data
		except:
			self.logger.error("Unexpected error: {:s}".format(sys.exc_info()[0]))

		if self.shmem_name != "":
			self.out_shmem_iface.send_ctr_terminate()
			time.sleep(2)
			self.out_shmem_iface.destory_sm_buffer()
			self.logger.info("Total dropped frames: {:d}".format(self.out_shmem_iface.dropped_frame_cntr))
		self.logger.info("Standard frame generator exited")

if __name__ == "__main__":
	logging.basicConfig(level=logging.DEBUG)
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
	generator = stdFrameGenator(frame_type, blocks, N_daq, data_type, shmem_name)
	generator.start()