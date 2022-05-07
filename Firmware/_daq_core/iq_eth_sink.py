"""
	Description :
	IQ Frame Ethernet receiver used for system level throughput testing
	
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
import logging
import socket
import time
# Import IQ header module
currentPath = os.path.dirname(os.path.realpath(__file__))
rootPath = os.path.dirname(currentPath)
sys.path.insert(0, os.path.join(rootPath, "_daq_core"))
from iq_header import IQHeader

class IQRecorder():
    def __init__(self, frame_count = 10000):
        """
            Initialzes the IQ Frame receiver module
        """
        self.logger = logging.getLogger(__name__)
        
        self.receiver_connection_status = False        
        self.port = 5000
        self.rec_ip_addr = "127.0.0.1"
        self.socket_inst = socket.socket()
        
        self.receiverBufferSize = 2 ** 18  # Size of the Ethernet receiver buffer measured in bytes        
        self.channel_number = 4
        self.sample_number = 2**18
        self.iq_header = IQHeader()
       
        # Misc parameters
        self.first_frame=0
        self.max_received_frames = frame_count        
        self.test_case = "phase_cont"
        self.dropped_frames = 0
        self.cpi_index_track = 0
        
    def connect_eth(self):                                    
        """
            Establish Ethernet connection to the IQ data interface

            Compatible only with DAQ firmwares that has the IQ streaming mode option. 
            HeIMDALL-DAQ Firmware version: 0.2 or later
        """
        try:
            self.socket_inst.connect((self.rec_ip_addr, self.port))            
            self.socket_inst.sendall(str.encode('streaming'))                                
            test_iq=self.receive_iq_frame()
            self.receiver_connection_status = True
            self.logger.info("Connection established")
        except:
            errorMsg = sys.exc_info()[0]
            self.logger.error("Error message: "+str(errorMsg))
            self.receiver_connection_status = False
            self.logger.info("Failed to connect to the IQ server")                
    
        
    def close_eth(self):  
        """
            Close the previously establed IQ data interface connection
        """          
        if self.receiver_connection_status:
            self.socket_inst.sendall(str.encode('q'))  # Quit from the server
            try:
                self.socket_inst.close()  # close connection   
                self.receiver_connection_status = False
                self.socket_inst = socket.socket()
            except:
                errorMsg = sys.exc_info()[0]
                self.logger.error("Error message: "+str(errorMsg))                
                return -1                 
        self.logger.info("Ethernet connection closed")
        return 0
        
    def listen(self):
        """
            Starts receiving IQ Frames through the IQ data interface
        """
        while not self.receiver_connection_status:
            time.sleep(1)
            self.connect_eth()
            
        self.logger.info("Starting IQ acquisition loop")        
        test_armed = False
        exit_flag = False
        while not exit_flag:
            
            # Check Rec control character                    
            try:
                self.socket_inst.sendall(str.encode("IQDownload")) # Send iq request command
                iq_frame_bytes = self.receive_iq_frame()
                if self.iq_header.check_sync_word():
                    self.logger.critical("Sync word error")
                    return -1

                #self.logger.info("IQ Frame received: {:d}".format(self.iq_header.cpi_index))

                if self.iq_header.delay_sync_flag == 1 and \
                    self.iq_header.iq_sync_flag == 1 and \
                    self.iq_header.frame_type == 0 and \
                    test_armed == False:
                    
                    test_armed = True
                    self.first_frame = self.iq_header.cpi_index
                    self.cpi_index_track = self.iq_header.cpi_index
                    self.logger.info("-->Armed<--")
                    self.logger.info("Start receiving: {:d} frames".format(self.max_received_frames))
                                        
                if test_armed:
                    
                    if self.cpi_index_track != self.iq_header.cpi_index:
                        frame_count_diff = self.iq_header.cpi_index-self.cpi_index_track
                        self.logger.error("{:d} frame droped, Exp.<->Rec. [{:d}<->{:d}]".format(frame_count_diff,self.cpi_index_track, self.iq_header.cpi_index))
                        self.dropped_frames += frame_count_diff
                        self.cpi_index_track = self.iq_header.cpi_index
                        
                    
                    self.cpi_index_track +=1                    
                    if self.first_frame + self.max_received_frames-1 <= self.iq_header.cpi_index:                        
                        exit_flag = True   
                        self.logger.info("Total dropped frames: {:d}".format(self.dropped_frames))
                        self.logger.info("First received frame: {:d}".format(self.first_frame))
                        self.logger.info("Current frame index: {:d}".format(self.iq_header.cpi_index))
            except:
                errorMsg = sys.exc_info()[0]
                self.logger.error("Error message: "+str(errorMsg))
                self.logger.error("Unable to get new IQ samples, exiting..")
                return -1
        self.close_eth()
        return 0
    
    def receive_iq_frame(self):
        """
                Receives IQ samples over Ethernet connection
        """        
        total_received_bytes = 0
        recv_bytes_count = 0
        iq_header_bytes = bytearray(self.iq_header.header_size)  # allocate array
        view = memoryview(iq_header_bytes)  # Get buffer
        
        self.logger.debug("Starting IQ header reception")
        
        while total_received_bytes < self.iq_header.header_size:
            # Receive into buffer
            recv_bytes_count = self.socket_inst.recv_into(view, self.iq_header.header_size-total_received_bytes)
            view = view[recv_bytes_count:]  # reset memory region
            total_received_bytes += recv_bytes_count
        
        self.iq_header.decode_header(iq_header_bytes)
        self.logger.debug("IQ header received and decoded")
        
        # Calculate total bytes to receive from the iq header data        
        total_bytes_to_receive = int((self.iq_header.cpi_length * self.iq_header.active_ant_chs * (2*self.iq_header.sample_bit_depth))/8)
        receiver_buffer_size = 2**18
        
        self.logger.debug("Total bytes to receive: {:d}".format(total_bytes_to_receive))
        
        total_received_bytes = 0
        recv_bytes_count = 0
        iq_data_bytes = bytearray(total_bytes_to_receive + receiver_buffer_size)  # allocate array
        view = memoryview(iq_data_bytes)  # Get buffer
        
        self.logger.debug("Starting IQ reception")
        
        while total_received_bytes < total_bytes_to_receive:
            # Receive into buffer
            recv_bytes_count = self.socket_inst.recv_into(view, receiver_buffer_size)
            view = view[recv_bytes_count:]  # reset memory region
            total_received_bytes += recv_bytes_count
        
        self.logger.debug("Succesfully received")
        
        # Dump IQ header        
        #self.iq_header.dump_header()
        # Enable here to get access to the received data
        #iq_frame_bytes=bytearray()+iq_header_bytes+iq_data_bytes[:total_bytes_to_receive]
        return 0#iq_frame_bytes

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    IQ_rec_inst0 = IQRecorder()    
    IQ_rec_inst0.listen()
    
