"""
	Description :
	Unit test for the sync module

	Project : HeIMDALL DAQ
	License : GNU GPL V3
	Author  : Tamas Peto

	Copyright (C) 2018-2020  Tamás Pető
	
	This program is free Falsesoftware: you can redistribute it and/or modify
	it under the terms of the GNU General Public License a  s published by
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

current_path      = dirname(realpath(__file__))
root_path         = dirname(dirname(current_path))
daq_core_path     = join(root_path, "_daq_core")
data_control_path = join(root_path, "_data_control")
log_path          = join(root_path, "_logs")
unit_test_path    = join(root_path, "_testing", "unit_test")
test_logs_path    = join(root_path, "_testing", "test_logs")


# Import IQ header module
sys.path.insert(0, daq_core_path)
from iq_header import IQHeader

class TesterSyncModule(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        logging.basicConfig(filename=join(test_logs_path,'unit_test.log'),
                             level=logging.INFO)   
        logging.info("--> Starting Sync module unit test <--")

        proc = subprocess.Popen(["rm",join(data_control_path,"sync_ctr")], stderr=subprocess.DEVNULL)
        proc.wait()
    
    def setUp(self):        
        # Set-up FIFOs   
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"sync_ctr")])
        proc.wait()
        
        # Open log files
        self.fd_log_gen_err  = open(join(log_path,"gen.log"),  "w")
        self.fd_log_sync_err = open(join(log_path,"sync.log"), "w")

        # Open sync control FIFO
        self.sync_ctr_fifo = open(join(data_control_path,"sync_ctr"), 'w+b', buffering=0)
        
        # Open test output file
        self.test_out = open(join(unit_test_path,'sync_test_0.dat'),"wb")
        
    def tearDown(self):
        # Close log files
        self.fd_log_gen_err.close()
        self.fd_log_sync_err.close()
        
        # Close sync control FIFO
        self.sync_ctr_fifo.close()
        proc = subprocess.Popen(["rm",join(data_control_path,"sync_ctr")], stderr=subprocess.DEVNULL)
        proc.wait()

        # Close test output file
        self.test_out.close()
        proc = subprocess.Popen(["rm",join(unit_test_path,'sync_test_0.dat')], stderr=subprocess.DEVNULL)
        proc.wait()

    #############################################
    #               TEST FUNCTIONS              #  
    #############################################

    def test_case_1_0(self):
        logging.info("-> Starting Test Case [1_0] : Frame forwared test with DATA frames")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA
        
        # -> Action <-
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_std_frame.py"),"-t",str(frame_type),"-b",str(frame_count)], stdout=subprocess.PIPE, stderr=self.fd_log_gen_err)
        gen_out, gen_err = generator.communicate()
        sync_module = subprocess.Popen([join(daq_core_path,"sync.out"),], stdin=subprocess.PIPE, stdout=self.test_out, stderr=self.fd_log_sync_err)
        sync_module.communicate(gen_out)
        sync_module.wait()
    
        # -> Assert <-
        # Note: 1 frame is utilized for buffering
        self.assertFalse(self.check_frame(join(unit_test_path,'sync_test_0.dat'), frame_count, frame_type))

    def test_case_1_1(self):
        logging.info("-> Starting Test Case [1_1] : Frame forwared test with DUMMY frames")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DUMMY
        
        # -> Action <-
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_std_frame.py"),"-t",str(frame_type),"-b",str(frame_count)], stdout=subprocess.PIPE, stderr=self.fd_log_gen_err)
        gen_out, gen_err = generator.communicate()
        sync_module = subprocess.Popen([join(daq_core_path,"sync.out"),], stdin=subprocess.PIPE, stdout=self.test_out, stderr=self.fd_log_sync_err)
        sync_module.communicate(gen_out)
        sync_module.wait()
        
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'sync_test_0.dat'), frame_count, frame_type))

    def test_case_1_2(self):
        logging.info("-> Starting Test Case [1_0] : Frame forwared test with CALIBRATION frames")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_CAL
        
        # -> Action <-
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_std_frame.py"),"-t",str(frame_type),"-b",str(frame_count)], stdout=subprocess.PIPE, stderr=self.fd_log_gen_err)
        gen_out, gen_err = generator.communicate()
        sync_module = subprocess.Popen([join(daq_core_path,"sync.out"),], stdin=subprocess.PIPE, stdout=self.test_out, stderr=self.fd_log_sync_err)
        sync_module.communicate(gen_out)
        sync_module.wait()
    
        # -> Assert <-
        # Note: 1 frame is utilized for buffering
        self.assertFalse(self.check_frame(join(unit_test_path,'sync_test_0.dat'), frame_count, frame_type))

    def test_case_1_10(self):
        logging.info("-> Starting Test Case [1_10] : ")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA

        # -> Action <-        
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_ramp.py"),"-t",str(frame_type),"-b",str(frame_count)], stdout=subprocess.PIPE, stderr=self.fd_log_gen_err)
        gen_out, gen_err = generator.communicate()
        sync_module = subprocess.Popen([join(daq_core_path,"sync.out"),], stdin=subprocess.PIPE, stdout=self.test_out, stderr=self.fd_log_sync_err)
        sync_module.communicate(gen_out)
        sync_module.wait()
        
        # -> Assert <-
        self.assertFalse(self.check_ramp(join(unit_test_path,'sync_test_0.dat'), [0,0,0,0]))

    def test_case_1_11(self):
        logging.info("-> Starting Test Case [1_11] : ")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_CAL

        # -> Action <-        
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_ramp.py"),"-t",str(frame_type),"-b",str(frame_count)], stdout=subprocess.PIPE, stderr=self.fd_log_gen_err)
        gen_out, gen_err = generator.communicate()
        sync_module = subprocess.Popen([join(daq_core_path,"sync.out"),], stdin=subprocess.PIPE, stdout=self.test_out, stderr=self.fd_log_sync_err)
        sync_module.communicate(gen_out)
        sync_module.wait()
        
        # -> Assert <-
        self.assertFalse(self.check_ramp(join(unit_test_path,'sync_test_0.dat'), [0,0,0,0]))

    def test_case_1_20(self):
        logging.info("-> Starting Test Case [1_20] : ")
        # -> Assume <-
        frame_count = 10
        frame_type = IQHeader.FRAME_TYPE_DATA
        delays = [2,5,1,7]
        M = 4
        
        # -> Action <-        
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_ramp.py"),"-t",str(frame_type),"-b",str(frame_count)], stdout=subprocess.PIPE, stderr=self.fd_log_gen_err)
        gen_out, gen_err = generator.communicate()
        sync_module = subprocess.Popen([join(daq_core_path,"sync.out"),], stdin=subprocess.PIPE, stdout=self.test_out, stderr=self.fd_log_sync_err)

        self.sync_ctr_fifo.write('d'.encode('ascii'))        
        self.sync_ctr_fifo.write(pack("i"*M,*delays))
            
        sync_module.communicate(gen_out)
        sync_module.wait()
        
        # -> Assert <-
        self.assertFalse(self.check_ramp(join(unit_test_path,'sync_test_0.dat'), delays))
    
    @unittest.skip("Reseting is not implemented")
    def test_case_1_21(self):
        logging.info("-> Starting Test Case [1_21] : ")
        # -> Assume <-
        frame_count = 10
        frame_type = IQHeader.FRAME_TYPE_DATA
        delays = [2,5,1,7]
        M = 4
        
        # -> Action <-        
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_ramp.py"),"-t",str(frame_type),"-b",str(frame_count)], stdout=subprocess.PIPE, stderr=self.fd_log_gen_err)
        gen_out, gen_err = generator.communicate()
        sync_module = subprocess.Popen([join(daq_core_path,"sync.out"),], stdin=subprocess.PIPE, stdout=self.test_out, stderr=self.fd_log_sync_err)
        # Set delay values
        self.sync_ctr_fifo.write('d'.encode('ascii'))        
        self.sync_ctr_fifo.write(pack("i"*M,*delays))
        time.sleep(1)
        # Reset delay values
        self.sync_ctr_fifo.write('r'.encode('ascii'))  
        delays = [0,0,0,0]  
        
        sync_module.communicate(gen_out)
        sync_module.wait()
        
        # -> Assert <-
        self.assertFalse(self.check_ramp(join(unit_test_path,'sync_test_0.dat'), delays))

    def test_case_1_100(self):
        logging.info("-> Starting Test Case [1_100] : Throughput testing")
        
        # -> Assume <-
        frame_count = 100
        
        # -> Action
        data_throughput_ratio = self._run_throughput_test(frame_count)
        
        # -> Assert <-
        self.assertTrue(data_throughput_ratio > 10)
        
    #############################################
    #              TEST RUN WRAPPERS            #  
    #############################################

    def _run_throughput_test(self,frame_count, sample_size=2**18):    
         
        # Prepare test vector               
        test_iq_frame_bytes = subprocess.Popen(["python3",join(unit_test_path,"gen_std_frame.py"),"-t",str(IQHeader.FRAME_TYPE_DATA),"-b",str(1)], 
                                               stdout=subprocess.PIPE, 
                                               stderr=self.fd_log_gen_err).communicate()[0]
        test_iq_frame_bytes = test_iq_frame_bytes*frame_count        
        
        # Start sync module
        sync_module = subprocess.Popen([join(daq_core_path,"sync.out"),], 
                                       stdin=subprocess.PIPE, 
                                       stdout=subprocess.DEVNULL, 
                                       stderr=self.fd_log_sync_err)
                
        #----------------Run speed test----------------------
        start_time = time.time()
        sync_module.communicate(test_iq_frame_bytes)       
        elapsed_time = time.time()-start_time
        #----------------Run speed test----------------------
        
        # -> Evaluation
        data_throughput  = (frame_count)*sample_size/10**6/elapsed_time
        data_throughput_ratio = data_throughput/2.4
        frame_throughput = (frame_count)/elapsed_time
        nominal_frame_throughput = 1/(sample_size/(2.4*10**6))

        logging.info("Sync module total runtime: {:.2f}".format(elapsed_time))
        logging.info("Total frame count:  {:d}".format(frame_count))
        logging.info("Throughput: {:.1f}/{:.1f} [MSps], {:.1f}/{:.1f} [frame/sec] - ".format(data_throughput,2.4,frame_throughput,nominal_frame_throughput))
        logging.info("Throughput ratio: {:.1f}".format(data_throughput_ratio))
        
        return data_throughput_ratio
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
                    if iq_header.frame_type != frame_type and blocks > 1:
                        logging.error("Unexpected frame type: {:d}".format(iq_header.frame_type))                        
                        return -2
                    
                    blocks+=1
                except EOFError:
                    pass
        if blocks == frame_count:
            return 0
        else:
            logging.error("Frame count error {:d}/{:d}".format(frame_count, blocks))
            return -3    
    
    def check_ramp(self, file_name, delays):
        iq_header = IQHeader()
        ramp_max = 29
        block_index = 0
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
                    
                    if block_index ==1:
                        time_index = int((iq_header.cpi_length/2)%ramp_max)
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
                    for m in range(iq_header.active_ant_chs):
                        ch_m_data_i = iq_data[m,0::2]
                        ch_m_data_q = iq_data[m,1::2]          
                        raw_sig_m = np.arange(time_index + delays[m], time_index + delays[m] + iq_header.cpi_length,1, dtype=np.uint32)%ramp_max
                        if (raw_sig_m[:]+2*m*ramp_max == ch_m_data_i).all() and (raw_sig_m[:]+(2*m+1)*ramp_max == ch_m_data_q).all(): 
                            pass                           
                        else:
                            return -1   
                    time_index+=iq_header.cpi_length
        return 0
