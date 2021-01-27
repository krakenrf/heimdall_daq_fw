"""
	Description : System level testing

	Project : HeIMDALL DAQ Firmware
	License : GNU GPL V3
	Author  : Tamas Peto
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


# Import IQ header module
sys.path.insert(0, daq_core_path)
from iq_header import IQHeader

# Import IQ frame sink module
from iq_eth_sink import IQRecorder

sys.path.insert(0, unit_test_path)
from gen_std_frame import stdFrameGenator

class TesterSystem(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        logging.basicConfig(filename=join(test_logs_path,'unit_test.log'),
                             level=logging.INFO)   
        logging.info("--> Starting IQ server test <--")
        
    def setUp(self):        
        warnings.simplefilter("ignore", ResourceWarning)
        self.logger = logging.getLogger(__name__)
        
        # Open log files
        self.fd_log_gen_err  = open(join(log_path,"gen.log"),  "w")
        self.fd_log_mut_err = open(join(log_path,"mut.log"), "w") # Module under test

    def tearDown(self):
        
        # Close log files
        self.fd_log_gen_err.close()
        self.fd_log_mut_err.close()
    
    #############################################
    #               TEST FUNCTIONS              #  
    #############################################
    @unittest.skip("Not implemented")
    def test_case_7_1(self):
        self.logger.info("-> Starting Test Case [7_1] : IQ server frame forward test")
        
        # -> Assume <-
        frame_type  = IQHeader.FRAME_TYPE_DATA
        frame_count = 100
        sample_size = 2**18
        
        # -> Action
        # Start the frame generator
        generator = stdFrameGenator(frame_type, frame_count+1, sample_size, 'CF32', 'delay_sync_iq')
        
        generator.iq_header.delay_sync_flag=1
        generator.iq_header.iq_sync_flag=1
        
        # Start the iq server module
        iq_server_module = subprocess.Popen([join(daq_core_path,"iq_server.out")],                                             
                                            stdout=subprocess.DEVNULL,
                                            stderr=self.fd_log_mut_err)
        
        # Start ethernet IQ frame receiver
        IQ_rec_inst0 = IQRecorder(frame_count)        
        time.sleep(2)                
        generator.start()
        IQ_rec_inst0.listen()
        self.logger.info("IQ frame reception interval finished")
    
        # -> Assert <-
        self.assertTrue(1)

    #############################################
    #              TEST RUN WRAPPERS            #  
    #############################################

    #############################################
    #         RESULT CHECKER FUNCTIONS          #  
    #############################################
    
    