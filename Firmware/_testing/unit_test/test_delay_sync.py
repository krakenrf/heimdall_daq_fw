"""
	Description :
	Unit test for the Delay synchronizer module

	Project : HeIMDALL DAQ Firmware
	License : GNU GPL V3
	Author  : Tamas Peto
	Status  : Under development
	Copyright (C) 2018-2020  Tamás Pető
	
	This program is free Falsesoftware: you can redistribute it and/or modify
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

current_path      = dirname(realpath(__file__))
root_path         = dirname(dirname(current_path))
daq_core_path     = join(root_path, "_daq_core")
data_control_path = join(root_path, "_data_control")
log_path          = join(root_path, "_logs")
unit_test_path    = join(root_path, "_testing", "unit_test")
test_logs_path    = join(root_path, "_testing", "test_logs")


# Import HeIMDALL modules
sys.path.insert(0, daq_core_path)
sys.path.insert(0, unit_test_path)
from iq_header import IQHeader
from capture_shmem_stream import IQFrameRecorder

class TesterDelaySyncModule(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        logging.basicConfig(filename=join(test_logs_path,'unit_test.log'),
                             level=logging.INFO)   
        logging.info("--> Starting Delay synchronizer unit test <--")
        try:
            # Close control FIFOs
            proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_out")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_out")], stderr=subprocess.DEVNULL)
            proc.wait()

            proc = subprocess.Popen(["rm",join(data_control_path,"fw_delay_sync_iq")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"bw_delay_sync_iq")], stderr=subprocess.DEVNULL)
            proc.wait()

            proc = subprocess.Popen(["rm",join(data_control_path,"fw_delay_sync_hwc")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"bw_delay_sync_hwc")], stderr=subprocess.DEVNULL)
            proc.wait()
        except (IOError, OSError):
            pass

        
    def setUp(self):        

        # Open log files
        self.fd_log_gen_err  = open(join(log_path,"gen.log"),  "w")
        self.fd_log_delay_sync_err = open(join(log_path,"delay_sync.log"), "w")

        # Set-up control FIFOs   
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"fw_decimator_out")])
        proc.wait()
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"bw_decimator_out")])
        proc.wait()

        proc = subprocess.Popen(["mkfifo",join(data_control_path,"fw_delay_sync_iq")])
        proc.wait()
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"bw_delay_sync_iq")])
        proc.wait()

        proc = subprocess.Popen(["mkfifo",join(data_control_path,"fw_delay_sync_hwc")])
        proc.wait()
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"bw_delay_sync_hwc")])
        proc.wait()
        
    def tearDown(self):
        # Close log files
        self.fd_log_gen_err.close()
        self.fd_log_delay_sync_err.close()
        
        # Close control FIFOs
        proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_out")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_out")], stderr=subprocess.DEVNULL)
        proc.wait()

        proc = subprocess.Popen(["rm",join(data_control_path,"fw_delay_sync_iq")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"bw_delay_sync_iq")], stderr=subprocess.DEVNULL)
        proc.wait()

        proc = subprocess.Popen(["rm",join(data_control_path,"fw_delay_sync_hwc")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"bw_delay_sync_hwc")], stderr=subprocess.DEVNULL)
        proc.wait()       

        # Remove test data file
        proc = subprocess.Popen(["rm", join(unit_test_path,'decimator_test_0.dat')], stderr=subprocess.DEVNULL)
        proc.wait()

        proc = subprocess.Popen(["rm", join(unit_test_path,'decimator_test_1.dat')], stderr=subprocess.DEVNULL)
        proc.wait()


    #############################################
    #               TEST FUNCTIONS              #  
    #############################################
           
    def test_case_6_100(self):
        logging.info("-> Starting Test Case [100] : Throughput testing")
        
        # -> Assume <-
        frame_count = 100
        
        # -> Action
        data_throughput_ratio = self._run_throughput_test(frame_count)
        
        
        # -> Assert <-
        self.assertTrue(data_throughput_ratio > 1.5)
        
    #############################################
    #              TEST RUN WRAPPERS            #  
    #############################################

    def _run_throughput_test(self,frame_count, sample_size=2**18):    
         
        # Prepare test vector               
        test_iq_frame_bytes = subprocess.Popen(["python3",join(unit_test_path,"gen_std_frame.py"),
                                                "-t", str(IQHeader.FRAME_TYPE_DATA),
                                                "-b", str(frame_count),
                                                "-d", "CF32",
                                                "-m", "decimator_out"], 
                                               stdout=subprocess.DEVNULL, 
                                               stderr=self.fd_log_gen_err)
        # Start delay sync module        
        delay_sync_module = subprocess.Popen(["python3",join(daq_core_path,"delay_sync.py")],                                        
                                       stdout=subprocess.DEVNULL, 
                                       stderr=self.fd_log_delay_sync_err)
        
        recorder_iq = IQFrameRecorder("delay_sync_iq",
                                      join(unit_test_path,'decimator_test_0.dat'),
                                      "0")
        if recorder_iq.in_shmem_iface.init_ok:
            recorder_iq.start()
        else:
            self.assertTrue(0)
        
        recorder_hwc = IQFrameRecorder("delay_sync_hwc",
                                      join(unit_test_path,'decimator_test_1.dat'),
                                      "0")
        if recorder_hwc.in_shmem_iface.init_ok:
            recorder_hwc.start()
        else:
            self.assertTrue(0)                      

        delay_sync_module.wait()
        recorder_iq.join()
        recorder_hwc.join()
               
        # -> Evaluation
        data_throughput  = (frame_count)*sample_size/10**6/recorder_iq.elapsed_time
        data_throughput_ratio = data_throughput/2.4
        frame_throughput = (frame_count)/recorder_iq.elapsed_time
        nominal_frame_throughput = 1/(sample_size/(2.4*10**6))

        logging.info("Decimator module total runtime: {:.2f}".format(recorder_iq.elapsed_time))
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
                    if iq_header.frame_type != frame_type:
                        logging.error("Unexpected frame type")
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
