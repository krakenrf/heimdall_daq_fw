"""
	Description :
	Unit test for the rebuffer module

	Project : HeIMDALL DAQ Firmware
	License : GNU GPL V3
	Author  : Tamas Peto
  
	Copyright (C) 2018-2021  Tamás Pető
	
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

# Import IQ header module
sys.path.insert(0, daq_core_path)
sys.path.insert(0, unit_test_path)
from iq_header import IQHeader
from capture_shmem_stream import IQFrameRecorder

class TesterRebufferModule(unittest.TestCase):


    @classmethod
    def setUpClass(cls):
        logging.basicConfig(filename=join(test_logs_path,'unit_test.log'),
                             level=logging.INFO)   
        logging.info("--> Starting Rebuffer unit test <--")

        try:
            proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_in")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_in")], stderr=subprocess.DEVNULL)
            proc.wait()
        except (IOError, OSError):
            pass
  
    def setUp(self):        
        # Open log files
        self.fd_log_gen_err  = open(join(log_path,"gen.log"),  "w")
        self.fd_log_rebuffer_err = open(join(log_path,"rebuffer.log"), "w")
        
        # Set-up control FIFOs   
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"fw_decimator_in")])
        proc.wait()
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"bw_decimator_in")])
        proc.wait()

                
        # Save default config file parameters
        self.N, self. R, self.N_daq, self.N_cal = self._read_config_file()
        
    def tearDown(self):
        # Close log files
        self.fd_log_gen_err.close()
        self.fd_log_rebuffer_err.close()        
        
        # Close control FIFOs        
        proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_in")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_in")], stderr=subprocess.DEVNULL)
        proc.wait()
        
        # Remove test output file
        proc = subprocess.Popen(["rm",join(unit_test_path,'rebuffer_test_0.dat')], stderr=subprocess.DEVNULL)
        proc.wait()
        
        # Write back default config file parameters
        self._write_config_file(self.N, self.R, self.N_daq, self.N_cal)

    #############################################
    #               TEST FUNCTIONS              #  
    #############################################
    #@unittest.skip("Skipped during development")
    def test_case_2_0(self):
        logging.info("-> Starting Test Case [2_0] :")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA
        self._write_config_file(N=2**18, R=1, N_daq=2**18, N_cal=2**18)
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)               
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'rebuffer_test_0.dat'), frame_count, frame_type))    
    def test_case_2_1(self):
        logging.info("-> Starting Test Case [2_1] :")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DUMMY
        self._write_config_file(N=2**18, R=1, N_daq=2**18, N_cal=2**18)
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)        
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'rebuffer_test_0.dat'), frame_count, frame_type))
    def test_case_2_2(self):
        logging.info("-> Starting Test Case [2_2] :")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_CAL
        self._write_config_file(N=2**18, R=1, N_daq=2**18, N_cal=2**18)
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'rebuffer_test_0.dat'), frame_count, frame_type))
    def test_case_2_3(self):
        logging.info("-> Starting Test Case [2_3] :")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA
        N     = 2**19
        R     = 1
        N_daq = 2**18
        N_cal = 2**20
        self._write_config_file(N,R,N_daq,N_cal)
        output_frame_count = (N_daq*frame_count)//(N*R)
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'rebuffer_test_0.dat'), output_frame_count, frame_type))
    def test_case_2_4(self):
        logging.info("-> Starting Test Case [2_4] :")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_CAL
        N     = 31    # Should be irrelevant
        R     = 10    # Should be irrelevant
        N_daq = 2**18
        N_cal = 2**20
        self._write_config_file(N,R,N_daq,N_cal)
        output_frame_count = (N_daq*frame_count)//(N_cal)
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'rebuffer_test_0.dat'), output_frame_count, frame_type))
    def test_case_2_5(self):
        logging.info("-> Starting Test Case [2_5] :")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA     
        N     = 101
        R     = 1
        N_daq = 37
        N_cal = 257 # Should irrelevant
        self._write_config_file(N,R,N_daq,N_cal)
        output_frame_count = (N_daq*frame_count)//(N*R)
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type,sample_size=N_daq)
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'rebuffer_test_0.dat'), output_frame_count , frame_type))
    def test_case_2_6(self):
        logging.info("-> Starting Test Case [2_6] :")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA     
        N     = 2**18
        R     = 3
        N_daq = 2**18   
        N_cal = 257 # Should irrelevant        
        self._write_config_file(N,R,N_daq,N_cal)
        output_frame_count = (N_daq*frame_count)//(N*R)
        # -> Action <-
        self._run_std_frame_test(frame_count, frame_type)
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'rebuffer_test_0.dat'), output_frame_count , frame_type))
    def test_case_2_10(self):
        logging.info("-> Starting Test Case [2_10] : Ramp test")
        # -> Assume <-
        frame_count = 10
        N     = 2**18
        R     = 1
        N_daq = 2**18
        N_cal = 257 # Should irrelevant  
        frame_type  = IQHeader.FRAME_TYPE_DATA
        output_frame_count = (N_daq*frame_count)//(N*R)
        self._write_config_file(N, R, N_daq, N_cal)        
        # -> Action <-        
        self._run_ramp_test(frame_count, frame_type)        
        # -> Assert <-
        self.assertFalse(self.check_ramp(join(unit_test_path,'rebuffer_test_0.dat'),output_frame_count))
    def test_case_2_11(self):
        logging.info("-> Starting Test Case [2_11] :")
        # -> Assume <-
        frame_count = 10
        N     = 31    # Should irrelevant  
        R     = 4     # Should irrelevant  
        N_daq = 2**18
        N_cal = 2**18
        frame_type  = IQHeader.FRAME_TYPE_CAL
        output_frame_count = (N_daq*frame_count)//(N_cal)
        self._write_config_file(N, R, N_daq, N_cal)
        # -> Action <-        
        self._run_ramp_test(frame_count, frame_type)
        # -> Assert <-
        self.assertFalse(self.check_ramp(join(unit_test_path,'rebuffer_test_0.dat'),output_frame_count))
    def test_case_2_12(self):
        logging.info("-> Starting Test Case [2_12] :")        
        # -> Assume <-
        frame_count = 1000
        frame_type  = IQHeader.FRAME_TYPE_DATA
        N     = 101
        R     = 2
        N_daq = 37
        N_cal = 257 # Should irrelevant  
        output_frame_count = (N_daq*frame_count)//(N*R)
        self._write_config_file(N,R,N_daq,N_cal)
        # -> Action <-        
        self._run_ramp_test(frame_count, frame_type, sample_size=N_daq)
        # -> Assert <-
        self.assertFalse(self.check_ramp(join(unit_test_path,'rebuffer_test_0.dat'), output_frame_count))

    #############################################
    #              TEST RUN WRAPPERS            #  
    #############################################
    
    def _run_std_frame_test(self,frame_count, frame_type, sample_size=2**18):        
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_std_frame.py"),
                                      "-t",str(frame_type),
                                      "-b",str(frame_count),
                                      "-n", str(sample_size)],
                                      stdout=subprocess.PIPE,
                                      stderr=self.fd_log_gen_err)
        gen_out, gen_err = generator.communicate()
        rebuffer_module = subprocess.Popen([join(daq_core_path,"rebuffer.out"), "0"], # 0-Drop mode disabled 
                                           stdin=subprocess.PIPE, 
                                           stdout=subprocess.DEVNULL, 
                                           stderr=self.fd_log_rebuffer_err)        
        recorder = IQFrameRecorder("decimator_in",
                                   join(unit_test_path,'rebuffer_test_0.dat'),
                                   "1")
        if recorder.in_shmem_iface.init_ok:
            recorder.start()
        else:
            self.assertTrue(0)
               
        rebuffer_module.communicate(gen_out)
        generator.wait()
        rebuffer_module.wait()
        recorder.join()
        

    def _run_ramp_test(self,frame_count, frame_type, sample_size=2**18):
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_ramp.py"),
                                      "-t",str(frame_type),
                                      "-b",str(frame_count),
                                      "-n",str(sample_size)],
                                      stdout=subprocess.PIPE,
                                       stderr=self.fd_log_gen_err)
        gen_out, gen_err = generator.communicate()
        rebuffer_module = subprocess.Popen([join(daq_core_path,"rebuffer.out"), "0"], # 0-Drop mode disabled 
                                           stdin=subprocess.PIPE, 
                                           stdout=subprocess.DEVNULL, 
                                           stderr=self.fd_log_rebuffer_err)
        recorder = IQFrameRecorder("decimator_in",
                                   join(unit_test_path,'rebuffer_test_0.dat'),
                                   "1")
        if recorder.in_shmem_iface.init_ok:
            recorder.start()
        else:
            self.assertTrue(0)

        rebuffer_module.communicate(gen_out)
        recorder.join()        
    #############################################
    #         RESULT CHECKER FUNCTIONS          #  
    #############################################
    
    def check_frame(self, file_name, frame_count, frame_type):
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
    
    def check_ramp(self, file_name, frame_count):
        iq_header = IQHeader()
        ramp_max = 29
        block_index = 0
        time_index = 0
        with open(file_name, "rb") as file_descr:
            while True:
                data_valid = False                
                block_index += 1                
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
                    iq_data_length = int((iq_header.cpi_length * iq_header.active_ant_chs * (2*iq_header.sample_bit_depth))/8)
                    # Reading Multichannel IQ data
                    iq_data_bytes = file_descr.read(iq_data_length)                    
                    if len(iq_data_bytes) ==0: break
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
                    iq_data = np.frombuffer(iq_data_bytes, dtype=np.uint8).reshape(iq_header.active_ant_chs, iq_header.cpi_length*2)            
                    raw_sig = np.arange(time_index, time_index + iq_header.cpi_length,1, dtype=np.uint32)%ramp_max
                    for m in range(iq_header.active_ant_chs):
                        ch_m_data_i = iq_data[m,0::2]
                        ch_m_data_q = iq_data[m,1::2]    
                        if (raw_sig[:]+2*m*ramp_max == ch_m_data_i).all() and (raw_sig[:]+(2*m+1)*ramp_max == ch_m_data_q).all(): 
                            pass                           
                        else:
                            logging.error("Ramp test failed!")
                            logging.error("Block: {:d}".format(block_index))
                            logging.error("Channel: {:d}".format(m))
                            for n in range(10):
                                logging.error("CH I ind:{:d}, [{:d}<->{:d}]"\
                                    .format(n,(raw_sig[:]+2*m*ramp_max)[n],ch_m_data_i[n]))
                                logging.error("CH Q ind:{:d}, [{:d}<->{:d}]"\
                                    .format(n,(raw_sig[:]+(2*m+1)*ramp_max)[n],ch_m_data_q[n]))
                            return -1   
                    time_index+=iq_header.cpi_length
            if block_index-1 != frame_count:
                logging.error("Frame count error occured in ramp test!")
                logging.error("Expected: {:d}, Received: {:d}".format(frame_count, block_index-1))
                return -2
            
        logging.info("Ramp test check {:d} frames".format(block_index-1))
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
            return 0,0,0
        N = parser.getint('pre_processing', 'cpi_size')        
        R = parser.getint('pre_processing', 'decimation_ratio')
        N_daq  = parser.getint('daq','daq_buffer_size')
        N_cal  = parser.getint('calibration', 'corr_size')
               
        return (N, R, N_daq, N_cal)
    
    def _write_config_file(self, N, R, N_daq, N_cal):
        """
        
        """
        parser = ConfigParser()
        found = parser.read([config_filename])
        if not found:            
            return -1
        
        parser['pre_processing']['cpi_size'] = str(N)
        parser['pre_processing']['decimation_ratio'] = str(R)       
        parser['daq']['daq_buffer_size'] = str(N_daq)
        parser['calibration']['corr_size'] = str(N_cal)
        with open(config_filename, 'w') as configfile:
            parser.write(configfile)
        return 0