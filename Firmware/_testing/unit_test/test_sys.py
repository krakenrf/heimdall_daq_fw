"""
	Description : System level testing

	Project : HeIMDALL DAQ
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

class TesterSystem(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        logging.basicConfig(filename=join(test_logs_path,'unit_test.log'),
                             level=logging.INFO)   
        logging.info("--> Starting System level testing <--")
        
    def setUp(self):        
        self.logger = logging.getLogger(__name__)
        
        # Open log files
        self.fd_log_gen_err  = open(join(log_path,"gen.log"),  "w")        

    def tearDown(self):
        
        # Close log files
        self.fd_log_gen_err.close()
    
    #############################################
    #               TEST FUNCTIONS              #  
    #############################################
    def test_case_0_200(self):
        self.logger.info("-> Starting Test Case [200] : System level throughput testing with synthetic data")
        
        # -> Assume <-
        frame_count = 10000
        
        # -> Action
        # Start the daq chain
        daq_chain_start = subprocess.Popen(["./daq_start_sm.sh"],
                                    stdout=subprocess.DEVNULL)
        daq_chain_start.wait()
        
        # Start ethernet IQ frame receiver
        IQ_rec_inst0 = IQRecorder(frame_count)        
        time.sleep(2)             
        IQ_rec_inst0.listen()
        self.logger.info("IQ frame reception interval finished")
        time.sleep(2)
        
        frame_drop_rate = IQ_rec_inst0.dropped_frames/IQ_rec_inst0.max_received_frames 
        self.logger.info("IQ Frame drop statistic:")
        self.logger.info("------------------------")
        self.logger.info("total / dropped frames: [{:d}/{:d}], drop rate {:.2f}".format(IQ_rec_inst0.max_received_frames,IQ_rec_inst0.dropped_frames,frame_drop_rate))

        # -> Assert <-
        self.assertTrue(IQ_rec_inst0.dropped_frames < 50)
        
    #############################################
    #              TEST RUN WRAPPERS            #  
    #############################################

    #############################################
    #         RESULT CHECKER FUNCTIONS          #  
    #############################################
    
    