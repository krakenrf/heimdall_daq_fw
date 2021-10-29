"""
	Description :
	Signal generator for testing the daq chain sample and phase 
	synchronization mechanism
	
	
	Project : HeIMDALL DAQ Firmware
	License : GNU GPL V3
	Author  : Tamas Peto

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
import time
import numpy as np
from struct import pack, unpack
from configparser import ConfigParser
import logging
from threading import Thread


# Import IQ header module
currentPath = os.path.dirname(os.path.realpath(__file__))
rootPath = os.path.dirname(currentPath)
sys.path.insert(0, os.path.join(rootPath, "_daq_core"))
from iq_header import IQHeader

# Import third party modules
from pyargus.directionEstimation import gen_scanning_vectors

#TODO: Use mutexes for the control variables
class FIFO_rd_thread(Thread):
    """
    Description:
    ------------
    This thread function handles the external requests using an external FIFO file.
    Upon receipt of a command, this thread infroms the main thread on the requested operation.
    
    The valid (1 byte) commands are the followings:
         r: Tuner reconfiguration
         n: Turning on the noise source
         f: Tunrning off the noise source
         g: Gain reconfiguration
         c: Center frequency change request
         2: Gentle system halt request
    """
    def __init__(self, ch_no):
        Thread.__init__(self)
        self.M = ch_no
        self.logger=logging.getLogger(__name__)
        self.ctr_fifo_descriptor = open("_data_control/rec_control_fifo", 'rb')
        if self.ctr_fifo_descriptor is not None:
            self.logger.debug("Control FIFO succesfully opened, waiting for ctr messages")
        else:
            self.logger.error("Failed to open control FIFO")
        
        # TODO: Do not store internal system parameters here
        self.gains = [0]*self.M
        self.rf_center_freq = 0
        self.noise_source_state = 0
        self.en_dummy_frame = 0
        
    def run(self):
        while(True):
            cmd = self.ctr_fifo_descriptor.read(1)
            self.logger.debug("Command character received: "+cmd.decode())
            
            if cmd.decode() == 'c':                
                self.logger.info("Ctr FIFO: Center frequency control message")
                center_freq_byte = self.ctr_fifo_descriptor.read(4)
                self.rf_center_freq = unpack("I", center_freq_byte)[0]                                                
                self.logger.debug("Center frequency: {:.2f} MHz".format(self.rf_center_freq/10**6))


            if cmd.decode() == 'g':                
                self.logger.info("Ctr FIFO: Gain control message")
                gain_bytes = self.ctr_fifo_descriptor.read(self.M*4)
                self.gains = unpack("I"*self.M, gain_bytes)                                
                for m in range(self.M):
                    self.logger.debug("Channel: {:d}, Gain:{:d} /10 dB".format(m, self.gains[m]))
                
            elif cmd.decode() == 'n':
                self.logger.info("Ctr FIFO: Enabling noise source")
                self.noise_source_state = 1

            elif cmd.decode() == 'f':
                self.logger.info("Ctr FIFO: Disabling noise source")
                self.noise_source_state = 0
            
            self.en_dummy_frame = 1


####################################
#           PARAMETERS 
####################################
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

parser = ConfigParser()
found = parser.read(['daq_chain_config.ini'])
if not found:
    logger.error("DAQ core configuration file not found. Exiting..")
    exit
logger.setLevel((parser.getint('daq', 'log_level')*10))

N = parser.getint('pre_processing', 'cpi_size')
N_daq = parser.getint('daq', 'daq_buffer_size')
M = parser.getint('hw', 'num_ch')
R = parser.getint('pre_processing', 'decimation_ratio')
fs  = parser.getint('daq', 'sample_rate') # Sampling frequency [Hz]
rf_freq = parser.getint('daq','center_freq') # Only used in the correspondig header field

# Synchronization related parameters
delays = [0]*M 
delays [1] = 10
delays [3] = 30

phase_diffs = [0]*M 
phase_diffs = [-10, 0, 20, 20, 40] # deg


cal_noise_phase_diffs = [0]*M
cal_noise_phase_diffs = [-10, 0, 20, 20, 40] # deg

power_diffs  = [0]*M# dB
power_diffs  = [0, -3, 3, 2, 1]# dB

blocks = 10000
block_size = N_daq*2    
sig_type = "noise" #"none" / "noise" / "cw" / "swept-cw" / "pulse"
sig_freq = 0.024 * 10**6# Interpreted as the distance from the center freq [Hz]
# For swept-CW source type:
sig_freq_sweep_start = -fs/2
sig_freq_sweep_stop  =  fs/2
sig_freq_sweep_step  =  fs/N*2**12
"""
    --> Power levels <--  

    Signal amplitude equals to 1 means that the ADC is full scaled
    and the IF gain is at maximum 49.6 dB. 
    Let us set call this power level to 0 dB.
    
"""
ps = -200  # Signal power [dB]
pn = -20  # Uncorrelated noise power [dB]
pow_noise_source_dB = -23 # Correlated noise signal from the internal source [dB]

"""
    --> Antenna layout <-- [Test Case 4]
"""
ant_config = "ULA"
ula_d = 0.5 # Inter element spacing [lambda]
uca_r = 1/(2*np.sqrt(2))

if ant_config == "ULA":
    ant_x = np.zeros(M)
    ant_y = np.arange(M) * ula_d   
    
else: # UCA
    ant_x = uca_r*np.cos(np.deg2rad(np.arange(M)*360/M)) 
    ant_y = -uca_r*np.sin(np.deg2rad(np.arange(M)*360/M))    

"""
    --> Test case control vector <--
    
    Put 1 in the corresponding position to enable a test case
    
    0: Phase diff is changed at the 30th data block
    1: Internal noise source phase diff is changed
    2: Dummy frame generation
    3: Increase power level of the usefull signal after the specified number of DAQ blocks
    4: Direction of arrival of the SOI is increased from block to block
    5: Squelch testing with one sinus bursts that has a length of one DAQ block
    6: Squelch testing with a short pulse
"""
test_case_ctr_vector    = [0]*7
test_case_ctr_vector[0] = 0
test_case_ctr_vector[1] = 0
test_case_ctr_vector[2] = 0
test_case_ctr_vector[3] = 0
test_case_ctr_vector[4] = 0
test_case_ctr_vector[5] = 0
test_case_ctr_vector[6] = 0



tc3_start = 100 # Block index when tc3 is initiated
tc4_start = 150 # Block index when tc4 is initiated
tc5_start = 150 # Block index when tc5 is initiated
tc6_start = 90 # Block index when tc6 is initiated

theta_tc4 = 90 # Initial angle for tc4

en_dummy_frame = False
dummy_frame_cntr = 0
no_dummy_frames = 5

####################################
#         Initialization
####################################
iq_header = IQHeader()

iq_header.sync_word            = IQHeader.SYNC_WORD
iq_header.header_version       = 7
iq_header.frame_type           = 0 # 0 - Normal data frame, 3 - calibration frame
iq_header.hardware_id          = "K"+str(M)
iq_header.unit_id              = 0              
iq_header.active_ant_chs       = M
iq_header.ioo_type             = 3             
iq_header.rf_center_freq       = 0       
iq_header.adc_sampling_freq    = fs
iq_header.sampling_freq        = fs
iq_header.cpi_length           = N_daq    
iq_header.time_stamp           = 0           
iq_header.daq_block_index      = 0   
iq_header.cpi_index            = 0            
iq_header.ext_integration_cntr = 0 
iq_header.data_type            = 2            
iq_header.sample_bit_depth     = 8     
iq_header.adc_overdrive_flags  = 0  
iq_header.if_gains             = [0]*32        
#iq_header.delay_sync_flag     = 0      
#iq_header.iq_sync_flag        = 0  
#iq_header.sync_state          = 0
iq_header.noise_source_state   = 0   
#iq_header.reserved=[0]*192       

logger.info("Decimation ratio: {:d}".format(R))
logger.debug("IQ header size: {:d}".format(len(iq_header.encode_header())))

FIFO_rd_thread_inst0 = FIFO_rd_thread(M)
FIFO_rd_thread_inst0.rf_center_freq = rf_freq
FIFO_rd_thread_inst0.start()

####################################
#            Simulation
####################################

# Allocation
signal = np.zeros((block_size), dtype=np.uint8) # This array stores the samples that are written to output
raw_sig_m = np.zeros((block_size//2+max(delays)), dtype = complex)
raw_sig_multiblock = np.zeros(N_daq*2, dtype=complex) # Stores 2 block of IQ samples
internal_noise_multiblock = np.zeros(N_daq*2, dtype=complex) # Stores 2 block of IQ samples
t_start = 4500 # For continuity testing
try:
    # Calculate signal amplitude
    sig_pow = 10**(ps/10)
    
    for b in range(blocks):        
        logger.info("Writing block: {:d}".format(b))
        iq_header.adc_overdrive_flags = 0
        iq_header.daq_block_index = b
        iq_header.time_stamp = int(time.time_ns()/10**6)
        # Generate signal of interest
        
        if sig_type == "noise":
            std_dev = np.sqrt(sig_pow/2)
            raw_sig = np.random.normal(0,std_dev,(N_daq))+1j*np.random.normal(0,std_dev,(N_daq))        
        elif sig_type == "cw":
            t = np.arange(N_daq) + t_start
            t_start = t[-1]
            sig_amp = np.sqrt(sig_pow)
            raw_sig =  sig_amp * np.exp(1j*2*np.pi*(sig_freq/fs)*t)
            logging.info("Current power : {:.2f} dB".format(ps))
        elif sig_type == "swept-cw":
            t = np.arange(N_daq)
            sig_amp = np.sqrt(sig_pow)
            logging.info("amp: {:.2f}".format(sig_amp))
            if b < tc3_start: 
                sig_freq = sig_freq_sweep_start
                sent_data_frames = 0 # Since the last frequency change
            elif en_dummy_frame == False:
                if sent_data_frames == 14: # 2+ decimation ratio 
                    sig_freq += sig_freq_sweep_step
                    en_dummy_frame = True        
                    dummy_frame_cntr = 0
                    sent_data_frames = 0                
                sent_data_frames += 1        
                
            if sig_freq > sig_freq_sweep_stop:
                sig_freq = sig_freq_sweep_start        
            logging.info("Current CW frequency: {:.2f}".format(sig_freq))
            raw_sig =  sig_amp * np.exp(1j*2*np.pi*(sig_freq/fs)*t)
        elif sig_type == "none":
            raw_sig = np.zeros(N_daq+max(delays), dtype=np.complex64)
        elif sig_type == "pulse":
            mask = np.zeros(N_daq, dtype=np.complex64)
            mask[0:100] = 1
            #std_dev = np.sqrt(sig_pow/2)
            #raw_sig = (np.random.normal(0,std_dev,(N_daq))+1j*np.random.normal(0,std_dev,(N_daq)))*mask
            t = np.arange(N_daq)             
            sig_amp = np.sqrt(sig_pow)
            raw_sig =  sig_amp * np.exp(1j*2*np.pi*(sig_freq/fs)*t)
            raw_sig*=mask
            


        # Generate coherent noise for calibration to simulate the internal noise source
        std_dev = np.sqrt(10**(pow_noise_source_dB/10)/2)
        internal_noise = np.random.normal(0,std_dev,(N_daq+max(delays)))+1j*np.random.normal(0,std_dev,(N_daq+max(delays)))

        # Arange coherent signal components in multiblock array
        start_index = b%2*(N_daq)
        raw_sig_multiblock[start_index:start_index+N_daq] = raw_sig[0:N_daq]
        internal_noise_multiblock[start_index:start_index+N_daq] = internal_noise[0:N_daq]
        
        # Dummy frame control
        if FIFO_rd_thread_inst0.en_dummy_frame:
            en_dummy_frame = True
            FIFO_rd_thread_inst0.en_dummy_frame = False
            dummy_frame_cntr = 0        
        
        # Pack coherent multichannel signal array
        for m in range(M):
            
            raw_sig_m = np.zeros((block_size//2), dtype = complex) 
            #####################
            # Test case control 
            #####################
            if m==0: # Chec and set only once
                if test_case_ctr_vector[0]:
                    if b == 50:
                        if m==0: logging.info("Activating test case 0")
                        phase_diffs=[0 , 50, 20, 30] # deg   
        
                if test_case_ctr_vector[1]:
                    if b == 70:
                        if m==0: logging.info("Activating test case 1")
                        cal_noise_phase_diffs=[0 , 0, 0, 0] # deg   
        
                if test_case_ctr_vector[2]:        
                    if (b==8):  
                        en_dummy_frame = True
                        dummy_frame_cntr = 0    
                    else:
                        en_dummy_frame = False
                        
                if test_case_ctr_vector[3]:
                    if (b==tc3_start):  
                        ps = -10
                        sig_pow = 10**(ps/10)

                if test_case_ctr_vector[4] and b > tc4_start:
                    if (tc4_start-b)%20 == 0:
                        theta_tc4 += 5
                        #theta_tc4 = 20
                    #phase_diffs = np.rad2deg(np.arange(M)*np.pi*np.cos(np.deg2rad(theta_tc4)))
                    phase_diffs = np.rad2deg(np.angle(gen_scanning_vectors(M, ant_x, ant_y, [theta_tc4,])[:,0]))
                    logging.info("DoA simulation - incident anlge :{:.2f}".format(theta_tc4))
                
                if test_case_ctr_vector[5] and b> tc5_start:
                    if b%20 == 0:
                        ps = -3
                        sig_pow = 10**(ps/10)
                    elif b%20 == 1:
                        ps = -200
                        sig_pow = 10**(ps/10)

                if test_case_ctr_vector[6] and b> tc6_start:
                    if b%20 == 0:
                        ps = -6
                        sig_pow = 10**(ps/10)
                        sig_type="pulse"
                    elif b%20 == 1:
                        sig_type="none"    
                        ps = -200
                        sig_pow = 10**(ps/10)
                
    
            #######################################
            # Compose received signal at channel m
            ####################################### 
            
            # Add Internal coherent noise source signal if the noise source is enabled
            if FIFO_rd_thread_inst0.noise_source_state == 1:
                if b%2 == 0:
                    raw_sig_m[0:delays[m]]     += internal_noise_multiblock[2*N_daq-delays[m]:2*N_daq] \
                                               * 10**(power_diffs[m]/20) \
                                               *  np.exp(1j*np.deg2rad(cal_noise_phase_diffs[m]))
                                               
                    raw_sig_m[delays[m]:N_daq] += internal_noise_multiblock[0:N_daq-delays[m]] \
                                               * 10**(power_diffs[m]/20) \
                                               *  np.exp(1j*np.deg2rad(cal_noise_phase_diffs[m]))
                else: # b%2 =1
                    raw_sig_m[:] += internal_noise_multiblock[N_daq-delays[m]:2*N_daq-delays[m]] \
                                 * 10**(power_diffs[m]/20) \
                                 *  np.exp(1j*np.deg2rad(cal_noise_phase_diffs[m]))
                
            # Add usefull signal component and perform amplitude and phase distortion
            if b%2 == 0:
                raw_sig_m[0:delays[m]]     += raw_sig_multiblock[2*N_daq-delays[m]:2*N_daq] \
                                           * 10**(power_diffs[m]/20) \
                                           *  np.exp(1j*np.deg2rad(phase_diffs[m]))
                                       
                raw_sig_m[delays[m]:N_daq] += raw_sig_multiblock[0:N_daq-delays[m]] \
                                           * 10**(power_diffs[m]/20) \
                                           *  np.exp(1j*np.deg2rad(phase_diffs[m]))
            else: # b%2 =1
                raw_sig_m[:] += raw_sig_multiblock[N_daq-delays[m]:2*N_daq-delays[m]] \
                             * 10**(power_diffs[m]/20) \
                             *  np.exp(1j*np.deg2rad(phase_diffs[m]))
            
            # # Corrupt useful signal with additive non-coherent noise 
            std_dev = np.sqrt(10**(pn/10)/2)
            noise = np.random.normal(0,std_dev,(N_daq))+1j*np.random.normal(0,std_dev,(N_daq))                       
            raw_sig_m += noise
                        
            #######################################
            # Test case [2] -Dummy frame generation
            #######################################   
            if en_dummy_frame and m==0:
                if dummy_frame_cntr == no_dummy_frames:
                    en_dummy_frame = False
                else:
                    dummy_frame_cntr +=1   
            if en_dummy_frame:                    
                raw_sig_m = np.zeros((N_daq), dtype=complex)                               
                       
            # Simulating ADC saturation
            raw_sig_m.real[raw_sig_m.real > 1] = 1
            raw_sig_m.imag[raw_sig_m.imag > 1] = 1
            
            raw_sig_m.real[raw_sig_m.real < -1] = -1
            raw_sig_m.imag[raw_sig_m.imag < -1] = -1
            
            #######################################
            #          IQ HEADER Control
            #######################################   
            if en_dummy_frame : # Dummy Frame
                iq_header.data_type  = 0
                iq_header.frame_type = IQHeader.FRAME_TYPE_DUMMY
                if m==0 :logging.info("Frame type: Dummy")  
            else:
                if FIFO_rd_thread_inst0.noise_source_state == 1: # Calibration Frame
                    iq_header.noise_source_state   = 1
                    iq_header.frame_type           = IQHeader.FRAME_TYPE_CAL
                    iq_header.data_type            = 1
                    if m==0 : logging.info("Frame type: Calibration")  
                else: # Normal Data Frame
                    iq_header.noise_source_state   = 0
                    iq_header.frame_type           = IQHeader.FRAME_TYPE_DATA
                    iq_header.data_type            = 1
                    if m==0 : logging.info("Frame type: Data")  
            
            # Update gain status in the header        
            iq_header.if_gains[m] = FIFO_rd_thread_inst0.gains[m]                  
            # Update center frequency field in the header
            iq_header.rf_center_freq = int(FIFO_rd_thread_inst0.rf_center_freq+sig_freq)            
            
            if (raw_sig_m.real == 1).any():
                iq_header.adc_overdrive_flags |= 1<<m
                logging.warning("Overdrive at channel: {:d}".format(m))
    
            if (raw_sig_m.imag == 1).any():
                iq_header.adc_overdrive_flags |= 1<<m
                logging.warning("Overdrive at channel: {:d}".format(m))
            
            #######################################
            #           SEND DATA FRAME
            ####################################### 
            
            raw_sig_m += (1+1j)
            raw_sig_m *= (255/2)
            
            signal[0::2] = raw_sig_m.real
            signal[1::2] = raw_sig_m.imag      
            byte_array= pack('B'*block_size, *signal)
            #logger.debug("Data block size: {:d} bytes".format(len(byte_array)))
            #iq_header.encode_header()
            #logger.debug("Header size: {:d}".format(len(iq_header.encode_header())))
            #iq_header.dump_header()
            # Pass IQ header and data to the next processing block		
            if m==0:            
                sys.stdout.buffer.write(iq_header.encode_header()) # Write the IQ header
                
            sys.stdout.buffer.write(byte_array) #Write IQ data
except:
    logging.error("Unexpected error: {:s}".format(sys.exc_info()[0]))
         
    
    
    
