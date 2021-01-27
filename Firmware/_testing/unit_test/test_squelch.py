"""
	Description :
	Unit test for the squelch module

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
import unittest
from os.path import join, dirname, realpath
import sys
import subprocess
import logging
import numpy as np
from struct import pack
import time
from configparser import ConfigParser

current_path      = dirname(realpath(__file__))
root_path         = dirname(dirname(current_path))
daq_core_path     = join(root_path, "_daq_core")
data_control_path = join(root_path, "_data_control")
log_path          = join(root_path, "_logs")
unit_test_path    = join(root_path, "_testing", "unit_test")
test_logs_path    = join(root_path, "_testing", "test_logs")

config_filename=join(root_path,'daq_chain_config.ini')

# Import HeIMDALL modules
sys.path.insert(0, daq_core_path)
sys.path.insert(0, unit_test_path)
from iq_header import IQHeader
from capture_shmem_stream import IQFrameRecorder
from gen_ramp import rampFrameGenator
from gen_std_frame import stdFrameGenator
from gen_burst import burstFrameGenator

class TesterSquelchModule(unittest.TestCase):


    @classmethod
    def setUpClass(cls):
        logging.basicConfig(filename=join(test_logs_path,'unit_test.log'),
                             level=logging.DEBUG)   
        logging.info("--> Starting Rebuffer unit test <--")
        try:
            proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_out")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_out")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"fw_squelch_out")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"bw_squelch_out")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"squelch_control_fifo")], stderr=subprocess.DEVNULL)
            proc.wait()
        except (IOError, OSError):
            pass
  

    def setUp(self):
        # Set-up FIFOs   
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"squelch_control_fifo")])
        proc.wait()
                
        # Open log files        
        self.fd_log_squelch_err = open(join(log_path,"squelch.log"), "w")
        
        # Set-up control FIFOs           
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"fw_decimator_out")])
        proc.wait()
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"bw_decimator_out")])
        proc.wait()
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"fw_squelch_out")])
        proc.wait()
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"bw_squelch_out")])
        proc.wait()

        # Save default config file parameters
        self.N, self.trig_th = self._read_config_file()
        
    def tearDown(self):
        # Close log files        
        self.fd_log_squelch_err.close()        
        
        # Remove test output file
        proc = subprocess.Popen(["rm",join(unit_test_path,'squelch_test_0.dat')])
        proc.wait()
        
        # Close control FIFOs                
        proc = subprocess.Popen(["rm",join(data_control_path,"squelch_control_fifo")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_out")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_out")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"fw_squelch_out")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"bw_squelch_out")], stderr=subprocess.DEVNULL)
        proc.wait()
        
        # Write back default config file parameters
        self._write_config_file(self.N, self.trig_th)

    #############################################
    #               TEST FUNCTIONS              #  
    #############################################
    
    def test_case_3_0(self):
        logging.info("-> Starting Test Case [3_0] : Frame forward test with normal data frames, squelch disabled")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA
        self._write_config_file(N=2**18, trig_th=0)     
        output_frame_count = frame_count   
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)               
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'squelch_test_0.dat'), output_frame_count, frame_type))
    
    def test_case_3_1(self):
        logging.info("-> Starting Test Case [3_1] : Frame forward test with normal dummy frames")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DUMMY
        self._write_config_file(N=2**18, trig_th=0)     
        output_frame_count = frame_count  
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)               
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'squelch_test_0.dat'), output_frame_count, frame_type))
    
    def test_case_3_2(self):
        logging.info("-> Starting Test Case [3_2] : Frame forward test with calibration frames")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_CAL
        self._write_config_file(N=2**18, trig_th=0)     
        output_frame_count = frame_count  
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)               
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'squelch_test_0.dat'), output_frame_count, frame_type))
    
    def test_case_3_3(self):
        logging.info("-> Starting Test Case [3_3] : Frame forward test with normal data frames, squelch enabled")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA
        self._write_config_file(N=2**18, trig_th=1)     
        output_frame_count = frame_count   
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)               
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'squelch_test_0.dat'), output_frame_count, IQHeader.FRAME_TYPE_TRIGW))
    
    def test_case_3_4(self):
        logging.info("-> Starting Test Case [3_4] : Frame forward test with calibration data frames, squelch enabled")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_CAL
        self._write_config_file(N=2**18, trig_th=1)     
        output_frame_count = frame_count
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)               
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'squelch_test_0.dat'), output_frame_count, frame_type))
    
    def test_case_3_5(self):
        logging.info("-> Starting Test Case [3_5] : Frame forward test with normal data frames, squelch enabled, threshold set")
        # -> Assume <-
        frame_count = 10        
        self._write_config_file(N=2**18, trig_th=0.5)     
        # -> Action <-
        self._run_squelch_test(test_case='3_5')
        # -> Assert <-        
        self.assertFalse(self.check_burst_test(join(unit_test_path,'squelch_test_0.dat')))
    
    def test_case_3_10(self):
        logging.info("-> Starting Test Case [3_10] : Burst ramp test")
        # -> Assume <-
        frame_count = 10        
        self._write_config_file(N=256, trig_th=0.5)     
        # -> Action <-
        self._run_squelch_test(test_case='3_10')
        # -> Assert <-        
        self.assertFalse(self.check_ramp(join(unit_test_path,'squelch_test_0.dat')))        
    
    @unittest.skip("Not implemented")
    def test_case_3_32(self):
        logging.info("-> Starting Test Case [3_32] :")
        pass
    #############################################
    #              TEST RUN WRAPPERS            #  
    #############################################
    def _run_std_frame_test(self,frame_count, frame_type, sample_size=2**18):        
        # Build up and start test chain
        generator = stdFrameGenator(frame_type, frame_count, sample_size, 'CF32', 'decimator_out')
        generator.start()

        squelch_module = subprocess.Popen([join(daq_core_path,"squelch.out"), "0"], # 0-Drop mode disabled                                            
                                           stdout=subprocess.DEVNULL,
                                           stderr=self.fd_log_squelch_err)
        recorder = IQFrameRecorder("squelch_out",
                                   join(unit_test_path,'squelch_test_0.dat'),
                                   "1")
        if recorder.in_shmem_iface.init_ok:
            recorder.start()
        else:
            self.assertTrue(0)              
        
        squelch_module.wait()
        generator.join()        
        recorder.join()
                
    def _run_squelch_test(self,test_case):
        generator = burstFrameGenator(test_case,"decimator_out")
        generator.start()
        squelch_module = subprocess.Popen([join(daq_core_path,"squelch.out"), "0"], # 0-Drop mode disabled                                            
                                           stdout=subprocess.DEVNULL,
                                           stderr=self.fd_log_squelch_err)
        recorder = IQFrameRecorder("squelch_out",
                                   join(unit_test_path,'squelch_test_0.dat'),
                                   "1")
        if recorder.in_shmem_iface.init_ok:
            recorder.start()
        else:
            self.assertTrue(0)
               
        squelch_module.wait()
        generator.join()   
        recorder.join()
    
    #############################################
    #         RESULT CHECKER FUNCTIONS          #  
    #############################################
    
    def check_burst_test(self, file_name):
        frame_count = 13
        iq_header = IQHeader()
        blocks = 0
        with open(file_name, "rb") as file_descr:
            while True:                                
                """
                ------------------
                  IQ Frame Reading
                ------------------
                """
                try:
                    # Reading IQ header
                    iq_header_bytes = file_descr.read(1024)                                          
                    if len(iq_header_bytes) == 0: break                               
                    iq_header.decode_header(iq_header_bytes)                                                              
                    # Reading Multichannel IQ data
                    iq_data_length = int((iq_header.cpi_length * iq_header.active_ant_chs * (2*iq_header.sample_bit_depth))/8)                    
                    if iq_data_length > 0:
                        iq_data_bytes = file_descr.read(iq_data_length)                    
                        if len(iq_data_bytes) ==0: break                    
                    
                    if iq_header.check_sync_word() !=0:
                        logging.error("Sync word error")
                        return -1
                    
                    # Frame type check
                    if blocks < 5 and (iq_header.frame_type != IQHeader.FRAME_TYPE_TRIGW):
                        logging.error("Unexpected frame type")
                        return -2
                    if blocks >= 5 and blocks <= 7 and (iq_header.frame_type != IQHeader.FRAME_TYPE_DATA):
                        logging.error("Unexpected frame type")
                        return -3
                    if blocks > 7 and (iq_header.frame_type != IQHeader.FRAME_TYPE_TRIGW):
                        logging.error("Unexpected frame type")
                        return -4
                    
                    blocks+=1
                except EOFError:
                    pass
        if blocks == frame_count:
            return 0
        else:
            logging.error("Frame count error [Expected/Received]: {:d}/{:d}".format(frame_count, blocks))
            return -5  
    
    def check_frame(self, file_name, frame_count, frame_type):
        """
            Checks the generated output of the module under test against the:
                - expected frame type
                - expected number of frames
                - IQ header sync word
        """
        iq_header = IQHeader()
        blocks = 0
        with open(file_name, "rb") as file_descr:
            while True:                                
                """
                ------------------
                  IQ Frame Reading
                ------------------
                """
                try:
                    # Reading IQ header
                    iq_header_bytes = file_descr.read(1024)                                          
                    if len(iq_header_bytes) == 0: break                               
                    iq_header.decode_header(iq_header_bytes)                                                        
                    # Reading Multichannel IQ data
                    iq_data_length = int((iq_header.cpi_length * iq_header.active_ant_chs * (2*iq_header.sample_bit_depth))/8)                    
                    if iq_data_length > 0:
                        iq_data_bytes = file_descr.read(iq_data_length)                    
                        if len(iq_data_bytes) ==0: break                    
                    
                    if iq_header.check_sync_word() !=0:
                        logging.error("Sync word error")
                        return -1
                    if iq_header.frame_type != frame_type:
                        logging.error("Unexpected frame type")
                        return -2
                    
                    blocks+=1
                except EOFError:
                    pass
        if blocks == frame_count:
            return 0
        else:
            logging.error("Frame count error [Expected/Received]: {:d}/{:d}".format(frame_count, blocks))
            return -3    
    
    def check_ramp(self, file_name):
        logging.info("Checking burst ramp")
        iq_header = IQHeader()        
        # -> Test case : 3_10 parameters
        ramp_max = 29        
        burst_1_len = 211
        burst_2_len = 197
        
        with open(file_name, "rb") as file_descr:
            while True:
                data_valid = False                                
                """
                ------------------
                  IQ Frame Reading
                ------------------
                """
                try:
                    # Reading IQ header
                    iq_header_bytes = file_descr.read(1024)                      
                    if len(iq_header_bytes) == 0: break                    
                    iq_header.decode_header(iq_header_bytes)
                    logging.debug("IQ frame read, type:{:d}, cpi index:{:d}".format(iq_header.frame_type,iq_header.daq_block_index))
                    #iq_header.dump_header() # For debug purposes
                    iq_data_length = int((iq_header.cpi_length * iq_header.active_ant_chs * (2*iq_header.sample_bit_depth))/8)
                    # Reading Multichannel IQ data
                    if iq_data_length > 0:
                        iq_data_bytes = file_descr.read(iq_data_length)                    
                        if len(iq_data_bytes) == 0: break
                        data_valid = True
                except EOFError:
                    pass                    
                """
                ------------------
                  Check Payload content
                ------------------
                """
                # Check payload content
                if data_valid:                
                    #iq_data = np.frombuffer(iq_data_bytes, dtype=np.uint8).reshape(iq_header.active_ant_chs, iq_header.cpi_length*2)
                    iq_cf64 = np.frombuffer(iq_data_bytes, dtype=np.complex64).reshape(iq_header.active_ant_chs, iq_header.cpi_length)
                    iq_cf64 = iq_cf64.copy()    
                                     
                    # -- > Block 0  < --
                    if iq_header.daq_block_index == 2:
                        raw_sig = np.arange(-1, iq_header.cpi_length-1,1, dtype=np.float32)%ramp_max                                                
                        raw_sig [burst_1_len::] = -1
                        raw_sig [0] = -1
                    # -- > Block 1  < --
                    elif iq_header.daq_block_index == 3:
                        raw_sig = np.arange(-1, iq_header.cpi_length-1,1, dtype=np.float32)%ramp_max                                                
                        raw_sig [burst_2_len::] = -1
                        raw_sig [0] = -1
                    # 0-ik minta 1 az 1es castornán a többin 0-kell legyen
                    mask = (raw_sig[:] >= 0).astype(int)          
                    for m in range(iq_header.active_ant_chs):                        
                        ch_m_data_i = iq_cf64[m,:].real
                        ch_m_data_q = iq_cf64[m,:].imag
                        ref_ramp_i = raw_sig[:] +  2*m*ramp_max 
                        ref_ramp_q = raw_sig[:] + (2*m+1)*ramp_max
                        diff_i = (ref_ramp_i - ch_m_data_i)*mask
                        diff_q = (ref_ramp_q - ch_m_data_q)*mask
                        if (diff_i < 0.1).all() and (diff_q < 0.1).all(): 
                            pass                           
                        else:
                            logging.error("Ramp test failed!")
                            logging.error("DAQ block index: {:d}".format(iq_header.daq_block_index))
                            logging.error("Channel: {:d}".format(m))
                            logging.error("Expected <--> Received")                            
                            for n in range(iq_header.cpi_length):
                                if diff_i[n] > 0.1 or diff_q[n] > 0.1:
                                    #logging.error("Diff I: {:f}, Diff Q: {:f}".format(diff_i[n], diff_q[n]))
                                    logging.error("CH I ind:{:d}, [{:f}<->{:f}]"\
                                        .format(n,(raw_sig[:]+2*m*ramp_max)[n],ch_m_data_i[n]))
                                    logging.error("CH Q ind:{:d}, [{:f}<->{:f}]"\
                                        .format(n,(raw_sig[:]+(2*m+1)*ramp_max)[n],ch_m_data_q[n]))                             
                            return -1
                        
        return 0

    #############################################
    #            AUXILIARY FUNCTIONS            #  
    #############################################

    def _read_config_file(self):
        """

        """
        parser = ConfigParser()
        found = parser.read([config_filename])
        if not found:            
            return 0,0,0,0
        N       = parser.getint('pre_processing', 'cpi_size')        
        trig_th = parser.getfloat('squelch','amplitude_threshold')
               
        return (N, trig_th)
    
    def _write_config_file(self, N, trig_th):
        """
        
        """
        parser = ConfigParser()
        found = parser.read([config_filename])
        if not found:            
            return -1
        
        parser['pre_processing']['cpi_size'] = str(N)
        parser['squelch']['amplitude_threshold'] = str(trig_th)
        with open(config_filename, 'w') as configfile:
            parser.write(configfile)
        return 0