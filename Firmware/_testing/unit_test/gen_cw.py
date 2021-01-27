"""
	Description :
	Swept CW signal generator for testing the decimator

	Project : HeIMDALL DAQ Firmware
	License : GNU GPL V3
	Author  : Tamas Peto

	Command line arguments:
		-t : Frame type (default:DATA)
		-b : Number of frames to send (default:10)
		-n : Number of samples per channel in the payload
  		-r : decimation ratio
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

# Import IQ header module
currentPath = os.path.dirname(os.path.realpath(__file__))
rootPath = os.path.dirname(os.path.dirname(currentPath))
sys.path.insert(0, os.path.join(rootPath, "_daq_core"))
from iq_header import IQHeader
from shmemIface import outShmemIface

####################################
#           PARAMETERS 
####################################
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

N_daq  = 2**18 
M      = 4 
R      = 1
blocks = 1000
frame_type = IQHeader.FRAME_TYPE_DATA
shmem_name = "decimator_in"

# Synchronization related parameters
block_size = N_daq*2
multi_frames = 1 # For continuity testing

# Generated excitation signal parameters
rf_freq = 100 *10**6 # Simulated center frequency  
fs      = 2.4 *10**6 # Sampling frequency [Hz]
sig_type = "cw" # "cw" / "swept-cw"
sig_freq = 0.024 * 10**6# Interpreted as the distance from the center freq [Hz]
ps       =   -3  # Signal power [dB]
pn       = - 50  # Uncorrelated noise power [dB]

# -> For swept-CW source type:
sig_freq_sweep_start = -fs/2
sig_freq_sweep_stop  =  fs/2
sig_freq_sweep_step  =  fs/N_daq*2**10

# Parse command line arguments
opts, args = getopt.getopt(sys.argv[1:],"b:n:r:s:m:")
for opt, arg in opts:
	if opt == '-b':
		blocks = int(arg)
	elif opt == '-n':
		N_daq = int(arg)
	elif opt == '-r':
		R = int(arg)
	elif opt == '-s':
 		sig_type= arg
	elif opt == '-m':
		shmem_name = arg


N_daq *= R
if sig_type=="cw":
    multi_frames = blocks
    sig_freq_sweep_start = fs/256
if sig_type=="swept-cw":
    blocks = int(fs / sig_freq_sweep_step)

 
logging.info("Frame type: {:d}".format(frame_type))
logging.info("Number of frames: {:d}".format(blocks))
logging.info("Number of sample per channel: {:d}".format(N_daq))
logging.info("Decimation ratio: {:d}".format(R))
logging.info("Signal source type: {:s}".format(sig_type))
if shmem_name != "":
	logging.info("Frames will be transfered on a shared memory interface")
	logging.info("Shared memory interface name: {:s}".format(shmem_name))


####################################
#         Initialization
####################################
iq_header = IQHeader()

iq_header.sync_word            = IQHeader.SYNC_WORD
iq_header.frame_type           = frame_type
iq_header.active_ant_chs       = M
iq_header.daq_block_index      = 0   
iq_header.cpi_index            = 0   
iq_header.cpi_length           = N_daq
iq_header.sample_bit_depth     = 8
iq_header.rf_center_freq       = 0       
iq_header.adc_sampling_freq    = int(fs)
iq_header.sampling_freq        = int(fs)
# Uninitialzed header fields are all zero!

if shmem_name != "":
    out_shmem_iface = outShmemIface(shmem_name,
                                 int(1024+N_daq*2*M*(iq_header.sample_bit_depth/8)),
                                 drop_mode = False)
    if not out_shmem_iface.init_ok:
        logging.critical("Shared memory initialization failed, exiting..")
        out_shmem_iface.destory_sm_buffer()
        exit()
    else:
    	logging.info("Shared memory interface succesfully initialized")

####################################
#            Simulation
####################################

# Allocation
signal_out  = np.zeros(N_daq*2, dtype=np.uint8) # This array stores the samples that are written to the output
signal_iqcf = np.zeros(N_daq, dtype=np.complex64)  # Stores the generate samples in complex float type
packet_start_time = 0
sig_pow = 10**(ps/10)
sig_amp = np.sqrt(sig_pow)
sig_freq = sig_freq_sweep_start
multi_frame_cntr = 0
try:
    for b in range(blocks):               
        logger.info("Writing block: {:d}".format(b))        
        logging.info("Current CW frequency: {:.2f} MHz".format(sig_freq/10**6))
        payload_byte_array = bytearray()
        
        # Generating time indexes
        t = np.arange(packet_start_time, packet_start_time+N_daq,1)
        packet_start_time += N_daq
        
        # Generating signal
        signal_iqcf =  sig_amp * np.exp(1j*2*np.pi*(sig_freq/fs)*t)        
        
        # Aditive noise to avoid spurs from quantization
        std_dev = np.sqrt(10**(pn/10)/2)
        noise = np.random.normal(0,std_dev,(N_daq))+1j*np.random.normal(0,std_dev,(N_daq))                       
        signal_iqcf += noise
        
        # Scaling and leveling      
        signal_iqcf += (1+1j)
        signal_iqcf *= (255/2)
        
        # Casting and packing    
        signal_out[0::2] = signal_iqcf.real
        signal_out[1::2] = signal_iqcf.imag      
        
        # Assembling payload
        for m in range(M):                
            payload_byte_array += pack('B'*N_daq*2, *signal_out)
        
        # Fill up header            
        iq_header.daq_block_index = b
        iq_header.rf_center_freq  = int(rf_freq+sig_freq)
        
        multi_frame_cntr +=1
        if multi_frame_cntr == multi_frames:
        	sig_freq += sig_freq_sweep_step
        	multi_frame_cntr = 0
        	packet_start_time = 0

        #######################################
        #           SEND DATA FRAME
        ####################################### 
        if shmem_name != "":
            active_buffer_index = out_shmem_iface.wait_buff_free()
            logging.info("Buffer free: {:d}".format(active_buffer_index))
            # Get the shared memory buffer
            iq_frame_buffer_out = (out_shmem_iface.buffers[active_buffer_index]).view(dtype=np.uint8)
            # Get the IQ sample array from the buffer
            iq_samples_out = iq_frame_buffer_out[1024:1024+iq_header.cpi_length*2*iq_header.active_ant_chs].reshape(iq_header.active_ant_chs, iq_header.cpi_length*2)
            if active_buffer_index !=3:
                (out_shmem_iface.buffers[active_buffer_index])[0:1024] = np.frombuffer(iq_header.encode_header(), dtype=np.uint8)
                for m in range(M):
                    iq_samples_out[m,0::2] = signal_iqcf[:].real
                    iq_samples_out[m,1::2] = signal_iqcf[:].imag
                out_shmem_iface.send_ctr_buff_ready(active_buffer_index)
        else:
            sys.stdout.buffer.write(iq_header.encode_header()) # Write the IQ header
            if (frame_type == IQHeader.FRAME_TYPE_DATA)  | (frame_type == IQHeader.FRAME_TYPE_CAL):
                sys.stdout.buffer.write(payload_byte_array) # Write IQ data
except:
    logging.error("Unexpected error: {:s}".format(sys.exc_info()[0]))
if shmem_name != "":
    out_shmem_iface.send_ctr_terminate()
    time.sleep(2)
    out_shmem_iface.destory_sm_buffer()
    logging.info("Total dropped frames: {:d}".format(out_shmem_iface.dropped_frame_cntr))
logging.info("Standard frame generator exited")

    
    
    
