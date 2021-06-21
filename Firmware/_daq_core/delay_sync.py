"""
   Delay Synchronizer Module for multichannel coherent receivers.
   
   Project: HeIMDALL DAQ Firmware
   Author: Tamas Peto
   License: GNU GPL V3
    
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
from struct import pack
import numpy as np
import numpy.linalg as lin
from configparser import ConfigParser
from iq_header import IQHeader
from shmemIface import outShmemIface, inShmemIface
from time import sleep

class delaySynchronizer():
    
    def __init__(self):

        self.scf_name = "_data_control/sync_control_fifo"
        self.sync_ctr_fifo = None
        self.in_shmem_iface = None
        self.in_shmem_iface_name = ""
        self.out_shmem_iface_iq = None
        self.out_shmem_iface_hwc = None
        
        self.logging_level = 0
        self.ignore_frame_drop_warning = True
        
        self.sync_delay_byte = 'd'.encode('ascii')
        self.sync_reset_byte = 'r'.encode('ascii')
        
        self.M = 8 # Number of receiver channels 
        self.N = 2**18 # Number of samples per channel
        self.R = 12 # Decimation ratio
        
        # Frame tracking
        self.expected_frame_index = -1
        
        # Calibration control parameters
        self.N_proc = 2**18        
        self.std_ch_ind = 0 # Index of standard channel. All channels are matched in delay to this one
        self.en_frac_cal = False # Enables fractional sample delay compensation, when decimation ratio is greather than 1
        self.en_iq_cal = False # Enables amlitude and phase calibration
                
        self.min_corr_peak_dyn_range = 20 # [dB]
        self.corr_peak_offset = 100 # [sample]
        self.cal_track_mode = 0        
        self.amplitude_cal_mode = "channel_power" # "default" / "disabled" / "channel_power"  -> Updated from .ini

        self.phase_diff_tolerance = 3 # deg, maximum allowable phase difference
        self.amp_diff_tolerance = 0.5 # power ratio  maximum allowable amplitude difference, not dB!
        
        self.sync_failed_cntr = 0 # Counts the number of iq or sample sync fails in track mode
        self.max_sync_fails = 3 # Maximum number of synchronization fails before the sync track is lost
        self.sync_failed_cntr_total = 0
        
        # Auxiliary state variables
        self.sample_compensation_cntr = 0 # Count the number of issued delay compensations
        self.iq_compensation_cntr = 0 # Count the number of issued iq compensations 
        self.last_update_ind=-3 # Hold the last index when the compensation has sent
        self.last_rf = 0 # Tracks the RF center frequency, recalibration is initiated when changed 
                
        # Overwrite default configuration
        self._read_config_file("daq_chain_config.ini")

        # Initialize logger        
        logging.basicConfig(level=self.logging_level)
        self.logger = logging.getLogger(__name__)
        float_formatter = "{:.2f}".format
        np.set_printoptions(formatter={'float_kind':float_formatter})
        
        self.iq_header = IQHeader()
        """
            Block sizes measured in bytes        
            1 IQ sample consist of 2 32bit float number
        """        
        self.in_block_size = self.N *self.M * 2 * 4
        
        self.logger.info("Antenna channles {:d}".format(self.M))
        self.logger.info("IQ samples per channel {:d}".format(self.N))  
        self.current_state = "STATE_INIT" 
        
        # List of the channels to be mathced 
        self.channel_list=(np.arange(self.M).tolist())
        self.channel_list.remove(self.std_ch_ind)        
        
        # Allocations
        self.corr_functions = np.zeros((self.M, self.N_proc*2))
        self.delays = np.zeros(self.M, dtype=int) # Holds the calculated samples delay
        self.iq_corrections = np.ones(self.M, dtype=np.complex64) # This vector holds the IQ compensation values
        self.iq_diff_ref = np.ones(self.M, dtype=np.complex64) # Reference IQ difference vector used in the tracking mode
        
        self.logger.info("Delay synchronizer initialized")
    
    def _read_config_file(self, config_filename):
        """
            Configures the internal parameters of the processing module based 
            on the values set in the confiugration file.

            TODO: Handle configuration field read failure
            Parameters:
            -----------
                :param: config_filename: Name of the configuration file
                :type:  config_filename: string
                    
            Return values:
            --------------
                :return: 0: Confiugrations fields succesfully applied
                        -1: Configuration file not found
        """
        parser = ConfigParser()
        found = parser.read([config_filename])
        if not found:
            self.logger.error("DAQ core configuration file not found. Default parameters will be used!")
            return -1
        self.N = parser.getint('pre_processing', 'cpi_size')
        self.M = parser.getint('hw', 'num_ch')
        self.R = parser.getint('pre_processing', 'decimation_ratio')
        self.N_proc = parser.getint('calibration', 'corr_size')
        self.std_ch_ind = parser.getint('calibration','std_ch_ind')
        self.amp_diff_tolerance = parser.getint('calibration', 'amplitude_tolerance')
        self.phase_diff_tolerance = parser.getint('calibration', 'phase_tolerance')
        self.cal_track_mode = parser.getint('calibration','cal_track_mode')
        self.max_sync_fails = parser.getint('calibration','maximum_sync_fails')
        self.amplitude_cal_mode = parser.get('calibration','amplitude_cal_mode')
        
        if parser.getint('calibration', 'en_frac_cal'):
            self.en_frac_cal = True
        else:
            self.en_frac_cal = False
        if parser.getint('calibration', 'en_iq_cal'):
            self.en_iq_cal = True
        else:
            self.en_iq_cal = False
        
        if parser.getint('squelch', 'en_squelch'):
            self.in_shmem_iface_name = "squelch_out"
        else:
            self.in_shmem_iface_name = "decimator_out"
        
        self.logging_level=(parser.getint('daq', 'log_level')*10)

        # Convert to voltage ratio
        self.amp_diff_tolerance = 10**(self.amp_diff_tolerance/20)
        
        return 0
    def open_interfaces(self):
        """
            Opens the communication interfaces of the module including the
            input and output shared memory interfaces and the FIFO control interface.
            
            Input shared memory interface: IQ data from the decimator/squelch module
            Out shared memory interfaces: Towards the IQ Server or the DSP module and to
            the Hardware Controller module.

            Through the Control FIFO interface the module sends delay compensation values 
            to the sync module.

            Return values:
            --------------
                :return: 0: All interfaces have been succesfully initialized
                        -1: Failed to initialize one the interfaces
        """ 
        
        # Open Sync-control FIFO        
        try:          
            self.sync_ctr_fifo = open(self.scf_name, 'w+b', buffering=0)            
        except OSError as err:
            self.logger.critical("OS error: {0}".format(err))
            self.logger.critical("Failed to open sync control fifo")            
            return -1
        
        # Open shared memory interface to receive data from the decimator
        self.in_shmem_iface = inShmemIface(self.in_shmem_iface_name)
        if not self.in_shmem_iface.init_ok:
            self.logger.critical("Shared memory (Decimator) initialization failed, exiting..")
            return -1
        
        # Open shared memory interface towards the iq server module
        self.out_shmem_iface_iq = outShmemIface("delay_sync_iq",
                                 int(1024+self.N*2*self.M*(32/8)),
                                 drop_mode = True)
        if not self.out_shmem_iface_iq.init_ok:
            self.logger.critical("Shared memory (IQ server) initialization failed, exiting..")
            return -1

        # Open shared memory interface towards the hardware controller module
        self.out_shmem_iface_hwc = outShmemIface("delay_sync_hwc",
                                 int(1024+self.N*2*self.M*(32/8)),
                                 drop_mode = True)
        if not self.out_shmem_iface_hwc.init_ok:
            self.logger.critical("Shared memory (HWC) initialization failed, exiting..")
            return -1
        return 0
    
    def close_interfaces(self):
        """
            Close the communication and data interfaces that are opened during the start of the module
        """
        if self.sync_ctr_fifo is not None:
            self.sync_ctr_fifo.write(pack('B',2) )
            self.sync_ctr_fifo.close()

        if self.in_shmem_iface is not None:
            self.in_shmem_iface.destory_sm_buffer()
                
        if self.out_shmem_iface_iq is not None:
            self.out_shmem_iface_iq.send_ctr_terminate()
            sleep(2)
            self.out_shmem_iface_iq.destory_sm_buffer()        

        if self.out_shmem_iface_hwc is not None:
            self.out_shmem_iface_hwc.send_ctr_terminate()
            sleep(2)
            self.out_shmem_iface_hwc.destory_sm_buffer()  
            
        self.logger.info("Interfaces are closed")
    
    def calc_sync(self, iq_samples):
        """
            This function calculates the synchronization status of the signal processing channels.
            
            Implementation notes:
            ---------------------
            It checks the sample level synchrony with calculating the cross correlation fucntion of 
            all the channels at two points. At zero and at non-zero offsets. In case the value obtained
            at zero offset is not remarkably higher than the value calculated at non-zero offet, we can
            consider the channels to be misaligned. 

            The amplitude and phase offsets are determined from the eigendecomposition of the spatial-correlation matrix

            Parameters:
            -----------
                :param: iq_samples: Processed IQ samples  (May contain less samples than what can be found in a frame)
                :type : iq_samples: Complex 2D numpy array
            
            Return values:
            --------------
                :return: dyn_ranges: Estimated peak-to-sidelobe ratio of the cross-correlation functions
                :return: iq_diffs  : Amplitude and Phase differences across the channels
                :rtype : dyn_ranges: list of floats
                :rtype : iq_diffs  : Complex 1D numpy array
                
        """
        iq_diffs   = np.ones(self.M, dtype=np.complex64)
        dyn_ranges = []        

        # Calculate cross-correlations to check sample level synchrony
        for m in self.channel_list:
            # Correlation at zero offset 
            iq_diffs[m]     = self.N_proc / (np.dot(iq_samples[m, :], 
                                                  iq_samples[self.std_ch_ind, :].conj()))
            # Correlation at the spcified offset
            corr_at_offset_m =  self.N_proc / (np.dot(iq_samples[m, self.corr_peak_offset::],
                                                   iq_samples[self.std_ch_ind, 0:-self.corr_peak_offset].conj()))
            # Check dynamic range
            dyn_ranges.append(-20*np.log10(abs(iq_diffs[m]) / abs(corr_at_offset_m)))

        # Calculate Spatial correlation matrix to determine amplitude-phase missmatches         
        Rxx = iq_samples.dot(np.conj(iq_samples.T))
        # Perform eigen-decomposition
        eigenvalues, eigenvectors = lin.eig(Rxx)
        # Get dominant eigenvector
        max_eig_index = np.argmax(np.abs(eigenvalues))
        vmax  = eigenvectors[:, max_eig_index] 
        iq_diffs = 1 / vmax
        iq_diffs /= iq_diffs[self.std_ch_ind]

        # Amplitude correction -  scaling IQ diferences
        if self.amplitude_cal_mode == "channel_power":
            channel_powers = list(map(lambda ch_ind: np.dot(iq_samples[ch_ind, :], iq_samples[ch_ind, :].conj())/self.N_proc, np.arange(self.M)))
            iq_diffs       = np.array(list(map(lambda m: iq_diffs[m]/np.abs(iq_diffs[m])*
                                                         np.sqrt(channel_powers[self.std_ch_ind]/channel_powers[m]),
                                           np.arange(self.M))))
        elif self.amplitude_cal_mode == "disabled":            
            iq_diffs        = np.array(list(map(lambda m: iq_diffs[m]/np.abs(iq_diffs[m]), np.arange(self.M))))
    
            return iq_diffs

        for m in range(self.M):
            self.logger.debug("Channel: {:d}, Peak dyn. range: {:.2f}[min: {:.2f}], Amp.:{:.2f}, Phase:{:.2f} ".format(\
                            m, dyn_ranges[-1], self.min_corr_peak_dyn_range, 20*np.log10(abs(iq_diffs[m])), 
                            np.rad2deg(np.angle(iq_diffs[m]))))  

        return np.array(dyn_ranges), iq_diffs
    def start(self):
        """
            Start the main processing loop
        """
        while True:
            sample_sync_flag = False
            iq_sync_flag     = False
            sync_state       = 0

            #############################################
            #           OBTAIN NEW DATA FRAME           #  
            #############################################
            
            # Acquire data
            active_buff_index_dec = self.in_shmem_iface.wait_buff_free()  
            if active_buff_index_dec < 0 or active_buff_index_dec > 1:
                self.logger.critical("Failed to acquire new data frame, exiting..")
                break;          
            iq_frame_buffer_in = self.in_shmem_iface.buffers[active_buff_index_dec]

            # Read and convert header
            iq_header_bytes = iq_frame_buffer_in[0:1024].tobytes()
            self.iq_header.decode_header(iq_header_bytes)            
            #self.iq_header.dump_header()
            
            if self.iq_header.check_sync_word():
                self.logger.critical("IQ header sync word check failed, exiting..")
                break
            
            # Initialize frame tracker
            if self.expected_frame_index == -1:
                self.expected_frame_index = self.iq_header.daq_block_index
            
            if self.expected_frame_index != self.iq_header.daq_block_index:
                if not self.ignore_frame_drop_warning: self.logger.warning("Frame index missmatch. Expected {:d} <--> {:d} Received"\
                    .format(self.expected_frame_index, self.iq_header.daq_block_index))
                self.expected_frame_index = self.iq_header.daq_block_index                
            
            self.expected_frame_index += self.R
            
            # Prepare payload buffer
            incoming_payload_size = self.iq_header.cpi_length*self.iq_header.active_ant_chs*2*int(self.iq_header.sample_bit_depth/8)
            if incoming_payload_size > 0:
                iq_samples_in = (iq_frame_buffer_in[1024:1024 + incoming_payload_size].view(dtype=np.complex64))\
                                .reshape(self.iq_header.active_ant_chs, self.iq_header.cpi_length)
                
            # Get buffers from the sink blocks (IQ server, HW controller)
            active_buffer_index_iq = self.out_shmem_iface_iq.wait_buff_free()
            active_buffer_index_hwc = self.out_shmem_iface_hwc.wait_buff_free() 
            
            self.logger.debug("Type:{:d}, CPI: {:d}, State:{:s}".format(
                    self.iq_header.frame_type, 
                    self.iq_header.cpi_index, 
                    self.current_state))
            #############################################
            #  Delay Synchronizer Finite State Machine  #
            #############################################
            
            if (self.iq_header.frame_type != IQHeader.FRAME_TYPE_DUMMY):  # Check frame type

                # -> IQ Preprocessing <-
                # TODO: Check payload size     
                if incoming_payload_size > 0:            
                    if active_buffer_index_iq !=3:
                        iq_frame_buffer_out = (self.out_shmem_iface_iq.buffers[active_buffer_index_iq]).view(dtype=np.complex64)
                        # IQ header offset:1 sample -> 8 byte, 1024 byte length header -> 128 "sample"
                        iq_samples_out = iq_frame_buffer_out[128:128+self.iq_header.cpi_length*self.iq_header.active_ant_chs].reshape(self.iq_header.active_ant_chs, self.iq_header.cpi_length)
                        
                        # Remove DC and apply IQ correction
                        for m in range(self.M):
                            iq_samples_out[m,:] = (iq_samples_in[m,:]-np.average(iq_samples_in[m,:]))*self.iq_corrections[m]
                        
                    else:       
                        iq_samples_out = iq_samples_in.copy()                        
                        
                        # -> Remove DC
                        for m in range(self.M):
                            iq_samples_out[m,:] -= np.average(iq_samples_out[m,:])
                            
                        # -> Correct IQ differences
                        if self.en_iq_cal:
                            for m in self.channel_list:
                                iq_samples_out[m, :] *= self.iq_corrections[m]
                    
                    iq_samples = iq_samples_out[:,0:self.N_proc] # Cut for further processing                              
                #
                #------------------------------------------>
                #            
                if self.current_state == "STATE_INIT": 
                    sync_state = 1
                    # Reset IQ corrections
                    self.iq_corrections = np.ones(self.M, dtype=np.complex64) 
                    # Calibration frame                    
                    if self.iq_header.frame_type == IQHeader.FRAME_TYPE_CAL: 
                        self.current_state = "STATE_SAMPLE_CAL"
                        
                #
                #------------------------------------------>
                #
                elif self.current_state == "STATE_SAMPLE_CAL":
                    sync_state        = 2
                    sample_sync_flag  = True
                    delay_update_flag = 0
                    
                    # ->  Calculate correlation functions            
                    np_zeros = np.zeros(self.N_proc, dtype=np.complex64)
                    x_padd = np.concatenate([iq_samples[self.std_ch_ind, 0:self.N_proc], np_zeros])
                    x_fft = np.fft.fft(x_padd)
                    
                    for m in self.channel_list:
                        y_padd = np.concatenate([np_zeros, iq_samples[m, 0:self.N_proc]])
                        y_fft = np.fft.fft(y_padd)
                        self.corr_functions[m,:] = np.abs(np.fft.ifft(x_fft.conj() * y_fft))**2
                    # ->  Calculate sample delays (Not sub sample), check dynamic range
                    # WARNING: This dynamic range checking assumes dirac like coorelation peak                    
                    for m in self.channel_list:
                        peak_index = np.argmax(self.corr_functions[m, :])

                        # Check dynamic range
                        # TODO: Check overindexing
                        dyn_range = 10*np.log10(self.corr_functions[m, peak_index] / 
                                                 self.corr_functions[m, peak_index+self.corr_peak_offset])
                        if dyn_range < self.min_corr_peak_dyn_range:
                            self.logger.warning("Correlation peak dynamic range is insufficient to perform calibration")
                            self.logger.warning("Real value: {:.2f}, minimum: {:.2f}".format(dyn_range, self.min_corr_peak_dyn_range))
                            delay_update_flag = 0
                            sample_sync_flag = False # Sync can not be checked properly
                            break
                        
                        # Calculate sample offset
                        self.delays[m] = (self.N_proc - peak_index)*self.R
                        # Warning: Use this only until sub sample delay calibration is not implemented
                        if np.abs(self.delays[m]) >= self.R:
                            sample_sync_flag = False # Misalling detected
                            delay_update_flag=1
                        self.logger.debug("Channel {:d}, delay: {:d}".format(m, self.delays[m]))
                    # Set time delay 
                    if delay_update_flag:
                        self.logger.info("Sending delays compensations [{:d}]".format(self.iq_header.cpi_index))
                        delays = (-1*self.delays).tolist()
                        
                        self.sync_ctr_fifo.write(self.sync_delay_byte)
                        self.sync_ctr_fifo.write(pack("i"*self.M,*delays))
                        self.sample_compensation_cntr+=1  # Used to track how many delays compensations have we sent so far
                        self.last_update_ind=self.iq_header.cpi_index
                        self.current_state = "STATE_SYNC_WAIT"    
                    
                    if sample_sync_flag:
                        self.current_state = "STATE_IQ_CAL"
                #
                #------------------------------------------>
                #
                elif self.current_state == "STATE_SYNC_WAIT":
                    sync_state = 3
                    # Wait for at least 5 frames to update the previously sent delay values                
                    if (self.iq_header.cpi_index > self.last_update_ind+5):
                        self.current_state = "STATE_SAMPLE_CAL"
                    
                #
                #------------------------------------------>
                #
                elif self.current_state == "STATE_IQ_CAL":
                    sync_state          = 4
                    iq_corr_update_flag = False
                    sample_sync_flag    = True
                    iq_sync_flag        = True                
                    
                    if self.en_iq_cal:
                        dyn_ranges, iq_diffs = self.calc_sync(iq_samples)
                        
                        if (dyn_ranges < self.min_corr_peak_dyn_range).any():
                            self.logger.warning("Correlation peak dynamic range is insufficient to perform calibration")
                            for m in range(self.M-1):                        
                                self.logger.warning("Real value: {:.2f}, minimum: {:.2f}".format(dyn_ranges[m], self.min_corr_peak_dyn_range))                        
                            
                            sample_sync_flag = False # It seems that the sample sync has lost
                            iq_sync_flag = False
                            iq_corr_update_flag = False                            
                           
                        # Check IQ calibration necessity
                        elif (abs(np.rad2deg(np.angle(iq_diffs[m]))) > self.phase_diff_tolerance) or \
                             (abs(iq_diffs[m]) > self.amp_diff_tolerance):  
                            iq_corr_update_flag = True
                            self.logger.debug("Amplitude or phase differenceas are out of tolerance")                        
                        
                        # Update correction values if needed                
                        if iq_corr_update_flag:
                            iq_sync_flag = False
                            self.iq_compensation_cntr+=1  # Used to track how many iq compensations have we issued so far
                            self.logger.info("Updating IQ correction values")                                        
                            self.iq_corrections *= iq_diffs
                            self.logger.info("Amplitude differences: {0}".format(20*np.log10(np.abs(iq_diffs))))
                            self.logger.info("Phase differernces: {0}".format(np.rad2deg(np.angle(iq_diffs))))
                    
                    if not sample_sync_flag:
                        self.current_state = "STATE_SAMPLE_CAL"
                    
                    if sample_sync_flag and iq_sync_flag:                    
                        self.current_state = "STATE_TRACK_LOCK"  
                        if self.cal_track_mode == 2 and self.en_iq_cal:
                            self.iq_diff_ref[:] = iq_diffs[:]
                #
                #------------------------------------------>
                #
                elif self.current_state == "STATE_TRACK_LOCK":
                    sync_state       = 5
                    sample_sync_flag = True
                    iq_sync_flag     = True
                    # Wait here until the calibration frame is turned off
                    if self.iq_header.frame_type == IQHeader.FRAME_TYPE_DATA: # Normal data frame
                        dyn_ranges, iq_diffs = self.calc_sync(iq_samples)
                        if self.cal_track_mode == 1:
                            self.iq_diff_ref[:] = iq_diffs[:]
                        self.current_state = "STATE_TRACK"
                        self.last_rf = self.iq_header.rf_center_freq
                        
                #
                #------------------------------------------>
                #
                elif self.current_state == "STATE_TRACK":
                    sync_state = 6
                    # Caltrack mode 0: Calibration tracking is disabled
                    # Caltrack mode 1: Normal continous tracking
                    # Caltrack mode 2: Track only on calibration frames

                    if self.cal_track_mode == 1 or \
                       (self.cal_track_mode == 2 and self.iq_header.frame_type == IQHeader.FRAME_TYPE_CAL):

                        dyn_ranges, iq_diffs = self.calc_sync(iq_samples)

                        # Check sample sync loss
                        if (dyn_ranges < self.min_corr_peak_dyn_range).any():
                            self.logger.warning("Sample sync may lost")
                            sample_sync_flag = False
                        else:
                            sample_sync_flag = True

                        if self.en_iq_cal:
                            # Check IQ sync loss
                            if (abs(np.rad2deg(np.angle(iq_diffs/self.iq_diff_ref))) > self.phase_diff_tolerance).any() or \
                               (abs(iq_diffs/self.iq_diff_ref) > self.amp_diff_tolerance).any():
                                   iq_sync_flag = False
                                   self.logger.warning("IQ sync may lost")
                                   for m in range(self.M):
                                       self.logger.debug("Differences: Amplitude {:.2f}, Phase: {:.2f}".format(
                                               20*np.log10((abs(iq_diffs[m]/self.iq_diff_ref[m]))), 
                                               (abs(np.rad2deg(np.angle(iq_diffs[m]/self.iq_diff_ref[m]))))))
                            else:
                                iq_sync_flag = True

                        # Track loss control
                        if (not sample_sync_flag) or (self.en_iq_cal and (not iq_sync_flag)):
                            self.sync_failed_cntr +=1
                            self.sync_failed_cntr_total+=1
                        else:
                            self.sync_failed_cntr -=1                       
                        if self.sync_failed_cntr == self.max_sync_fails:
                            self.current_state = "STATE_INIT"
                            self.sync_failed_cntr = 0
                        elif self.sync_failed_cntr < 0: # Sync tracking holds
                            self.sync_failed_cntr = 0

                    else:
                        self.logger.debug("Sync flags are set")
                        sample_sync_flag = True
                        iq_sync_flag = True
                    
                    # Has the RF center frequency changed?
                    if self.last_rf != self.iq_header.rf_center_freq:
                        self.logger.info("Center frequency changed, initiating recalibration")
                        sample_sync_flag = False
                        iq_sync_flag = False
                        self.sync_failed_cntr = 0
                        self.current_state = "STATE_INIT"
    
                # Uncomment it for long term delay compenstation stress!
                self.logger.info("Delay track statistic [sync fails ,sample, iq, total][{:d},{:d},{:d}/{:d}]".format(
                                 self.sync_failed_cntr_total, 
                                 self.sample_compensation_cntr, 
                                 self.iq_compensation_cntr, 
                                 self.iq_header.daq_block_index))                                             
            
            #############################################   
            #         SEND PROCESSED DATA BLOCK         #  
            #############################################
            # -> Update header field
            if sample_sync_flag: 
                self.iq_header.delay_sync_flag=1
            else:
                self.iq_header.delay_sync_flag=0
            if iq_sync_flag:
                self.iq_header.iq_sync_flag=1
            else:
                self.iq_header.iq_sync_flag=0
            
            self.iq_header.sync_state = sync_state
            
            # -> Send IQ frame toward the iq server
            header_uint8 = np.frombuffer(self.iq_header.encode_header(), dtype=np.uint8)
            if active_buffer_index_iq !=3 :
                (self.out_shmem_iface_iq.buffers[active_buffer_index_iq])[0:1024] = header_uint8
                self.out_shmem_iface_iq.send_ctr_buff_ready(active_buffer_index_iq)
            else:
                if not self.ignore_frame_drop_warning: self.logger.warning("Dropping frame - IQ server, Total: {:d}".format(self.out_shmem_iface_iq.dropped_frame_cntr))

            # -> Send IQ frame toward the hwc module
            if active_buffer_index_hwc !=3 :
                (self.out_shmem_iface_hwc.buffers[active_buffer_index_hwc])[0:1024] = header_uint8
                # TODO: For ADPIS control HWC module should get informed about the power levels from the header
                self.out_shmem_iface_hwc.send_ctr_buff_ready(active_buffer_index_hwc)
            else:
                if not self.ignore_frame_drop_warning: self.logger.warning("Dropping frame - HWC, Total: {:d}".format(self.out_shmem_iface_hwc.dropped_frame_cntr))
            
            # -> Inform the preceeding block that we have finished the processing
            self.in_shmem_iface.send_ctr_buff_ready(active_buff_index_dec)
            
if __name__ == '__main__':
    delay_synchronizer_inst0 = delaySynchronizer()
    if delay_synchronizer_inst0.open_interfaces() == 0:
        delay_synchronizer_inst0.start()
    
    delay_synchronizer_inst0.close_interfaces()
