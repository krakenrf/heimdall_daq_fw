"""    
    Hardware Controller Module 

    Project  : HeIMDALL DAQ Firmware 
    Author   : Tamás Pető
    RF front : R820T compatible, For ADPIS dedicated IQ modulator is required
    License  : GNU GPL V3
        
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
# Import built in modules
from struct import pack, unpack
import threading
import logging

# Import third-party modules
import numpy as np
import socket
from configparser import ConfigParser

# Import HeIMDALL modules
from iq_header import IQHeader
from shmemIface import inShmemIface
import zmq
import inter_module_messages

# Global: Used to communicate between the HWC module and the Control Interface server
ctr_request = [] # This list stores the confiuration command and parameters [cmd, param 1, param 2, ..]
ctr_request_condition = threading.Condition()

class HWC():
    
    def __init__(self):                 
        
        logging.basicConfig(level=10)
        self.logger = logging.getLogger(__name__)
        self.log_level=0 # Set from the ini file        
        self.module_identifier = 6 # Inter-module message module identifier
        self.track_lock_ctr_fname = "_data_control/iq_track_lock"
        self.track_lock_ctr_fd = None
        self.in_shmem_iface = None
        # Gain index:       0  1   2   3   4   5   6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28
        self.valid_gains = [0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254, 280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496]
        # Defined by the R820T tuner
        
        self.cal_gain_table=np.array([[100,200,300,400,500,600,700,1700],[6, 10, 13, 14, 18, 22, 28, 28]]) # First column frequeny [MHz], second column gain index [valid_gains]
        self.cal_gain_table[0,:]*=10**6 # Convert to Hz
        self.M = 7 # Number of receiver channels 
        self.N = 2**18 # Number of samples per channel
        self.N_proc = 2**13
        self.iq_mod = None
        self.en_adpis= 0 # Enables or Disables of the ADPIS hardware usage 
        self.gains=[0]*self.M  # Initial gain values (Indices, not exact values!)
        self.noise_source_state = False
        self.gain_tune_states=[False]*self.M
        self.gain_lock_cntr = 0 # Auxiliary counter used for the gain lock
        self.gain_lock_interval = 0 # Required num. of frames to finish gain tuning
        self.unified_gain_control = True # Gain values will be equal for all the receivers
        self.last_gains=[0]*self.M # Stores the gain values to reset them after calibration
        
        # Support for calibration track mode 
        self.cal_track_mode = 0
        self.cal_frame_interval = 50 # Number of frames between two cal frames in cal_track_mode=2
        self.cal_frame_burst_size = 5 # Number of cal frames in a burst
        self.cal_frame_cntr = 0
        self.en_iq_cal = False
        self.sync_failed_cntr = 0 # Counts the number of iq or sample sync fails in track mode
        self.max_sync_fails = 3 # Maximum number of synchronization fails before the sync track is lost

        self.require_track_lock_intervention = True
        self.rf_center_frequency = 1

        # Overwrite default configuration
        self._read_config_file("daq_chain_config.ini")
        self.iq_header = IQHeader()
        
        self.current_state = "STATE_INIT" 
        """
            Block sizes measured in bytes        
            1 IQ sample consist of 2 32bit float number
        """
        # Configure logger                        
        self.logger.setLevel(self.log_level)

        # Control interface server
        self.ctr_iface_server = CtrIfaceServer(self.M)
        self.ctr_iface_server.start()

        self.logger.info("Antenna channles {:d}".format(self.M))
        self.logger.info("IQ samples per channel {:d}".format(self.N))        
        self.logger.info("Processing sample size: {:d}".format(self.N_proc))
        self.logger.info("ADPIS state: {:d}".format(self.en_adpis))
        self.logger.warning("Reference channel index is fixed 0 ")
        self.logger.info("Hardware Controller initialized")
        
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
        self.N_proc = parser.getint('adpis', 'adpis_proc_size')
        gains_init_str=parser.get('adpis','adpis_gains_init')
        self.cal_track_mode = parser.getint('calibration','cal_track_mode')        
        self.rf_center_frequency = parser.getint('daq','center_freq')
        self.max_sync_fails = parser.getint('calibration','maximum_sync_fails')
        self.cal_frame_burst_size = parser.getint('calibration','cal_frame_burst_size')
        self.cal_frame_interval = parser.getint('calibration','cal_frame_interval')
        self.gain_lock_interval = parser.getint('calibration','gain_lock_interval')     
        
        if parser.getint('calibration', 'unified_gain_control'):
            self.unified_gain_control=1
        else:
            self.unified_gain_control=0
        if parser.getint('calibration','en_gain_tune_init'):
            self.gain_tune_states=[True]*self.M
        else:
            self.gain_tune_states=[False]*self.M
        if parser.getint('adpis','en_adpis'):
            self.en_adpis=1
        else:
            self.en_adpis=0
        if parser.getint('calibration','require_track_lock_intervention'):
            self.require_track_lock_intervention=True
        else:
            self.require_track_lock_intervention=False
        if parser.getint('calibration', 'en_iq_cal'):
            self.en_iq_cal = True
        else:
            self.en_iq_cal = False
        self.log_level = parser.getint('daq','log_level')*10

        # Convert the gain list
        gains_init_str = gains_init_str.split(',')
        gains_init_ind = list(map(int, gains_init_str))
        # -> Channel number check
        if len(gains_init_ind) != self.M:
            logging.warning("Channel number missmatch when reading initial gain values")
            logging.warning("Gain values leaved in default state")
            self.gains=[0]*self.M
            self.last_gains=[0]*self.M
            logging.warning("GAINS: {0}".format(self.gains))
            logging.warning("Last GAINS: {0}".format(self.last_gains))
        else:
            self.gains=[]
            self.last_gains=[]
            try:
                for gain_item in gains_init_ind:
                    self.gains.append(self.valid_gains.index(gain_item))
                    self.last_gains.append(self.valid_gains.index(gain_item))
            except ValueError:
                logging.error("Improper inital gain values are given, set all to 0")
                self.gains=[0]*self.M
                self.last_gains=[0]*self.M

    def init(self):
        """
            Initializes the Hardware Controller module
                - Opens inter-module communication sockets
                - Opens the control FIFOs
                - Initializes the shared memory data interface
                - Initializes the DAC controller module (ADPIS control)
                - Sets initial IQ values in the ADPIS
        """
        # Open RTL-DAQ control socket
        context = zmq.Context()        
        self.rtl_daq_socket = context.socket(zmq.REQ)
        self.rtl_daq_socket.connect("tcp://localhost:1130")

        # Open control FIFOs
        try:            
            self.track_lock_ctr_fd = open(self.track_lock_ctr_fname, 'r')
        except OSError as err:
            self.logger.critical("OS error: {0}".format(err))
            self.logger.critical("Failed to open control fifos, exiting..")
            return -1        
        # Initialize shared memory interface
        self.in_shmem_iface = inShmemIface("delay_sync_hwc")
        if not self.in_shmem_iface.init_ok:
            logging.critical("Shared memory initialization failed")
            return -3
        # ADPIS
        if self.en_adpis:
            try:
                from dac_controller import DACController
            
                # Initialize DAC controller
                self.iq_mod = DACController(iface="I2C")
                if not self.iq_mod.init_status:
                    logging.critical("DAC Controller initialization failed")
                    return -2
                self.iq_mod.logger.setLevel(self.log_level)

                # Set inital IQ values
                for m in range(self.M-1):
                    self.iq_mod.set_IQ_value(0.5, 0.5, m)
            except:
                logging.error("DAC Controller initialization failed")
        return 0
    
    def close(self):
        """
            Close the communication and data interfaces that are opened during the start of the module
        """         
        if self.track_lock_ctr_fd is not None:
            self.track_lock_ctr_fd.close()
        
        if self.in_shmem_iface is not None:
            self.in_shmem_iface.destory_sm_buffer()
            
        self.logger.info("Module interfaces are closed")
    def _change_gains(self):
        """
            Sends gain tuning request to the receiver module through the 
            receiver configuration FIFO
            
            Used important object parameter:
            
            :param: self.gains: Gain index array
            :type: gains: list of integers [gain index0, gain index1 , ..]
        """
        if self.unified_gain_control:
            # Force the same gain value for all the channels
            self.gains = [min(self.gains)]*self.M
            
        # Prepare gain list
        gains=[]
        for m in range(self.M):
            gains.append(self.valid_gains[self.gains[m]])
            self.logger.info("Send Ch {:d} Gain: {:d} [{:d}]".format(m, int(gains[m]), self.iq_header.cpi_index))
        # Send gain list
        msg_byte_array = inter_module_messages.pack_msg_set_gain(self.module_identifier, gains)
        self.rtl_daq_socket.send(msg_byte_array)
        reply = self.rtl_daq_socket.recv()
        self.logger.debug(f"Received reply: {reply}")        
    def _tune_gains(self):
        """
            Performs IF gain tuning in order to maximaze the SINR in each channels by
            minimizing the effect of the quantization noise.
        """
        # Check gain states
        for m in range(self.M):
            if self.valid_gains[self.gains[m]] != self.iq_header.if_gains[m]:
                self.logger.error("Incosistent IF gains. Tuning is bypassed")
                return -1

        for m in range(self.M):
            # Check overdrive
            if self.iq_header.adc_overdrive_flags & 1<<m:
                self.logger.warning("ADC overdriven at channel: {:d}".format(m))
                if self.gains[m] != 0:
                    self.gains[m] -=1
                else:
                    self.logger.error("Overdrive detected, but the gain can not be decreased further")
                self.gain_tune_states[m]=False
                if self.unified_gain_control: # Disable tuning in all channels
                    self.gain_tune_states = [False]*self.M
            # Perform gain change if tuning is not finished
            if self.gain_tune_states[m]:
                if self.gains[m] < len(self.valid_gains)-1:
                    self.gains[m] +=1
                else:
                    self.logger.warning("Maximum gain reached, without overdrive, ch: {:d}".format(m))
                    self.gain_tune_states[m]=False
                    if self.unified_gain_control: # Disable tuning in all chanels
                        self.gain_tune_states = [False]*self.M 
        self._change_gains()

    def _handle_control_reqest(self, command, params):
        """
            Handles external control requests that has arrived thorugh the control Ethernet interface

            Parameters:
            -----------
            :param: command: String identifier of the requested configuration command 
            :param: params : List of parameters of the configuration command

            :type: command : string (4 character length)
            :type: params  : python list

            Implemented valid command strings:
            - FREQ: Changes the center frequency of the receiver
            - GAIN: Sets the IF gain values

        """
        if command == "FREQ":            
            msg_byte_array = inter_module_messages.pack_msg_rf_tune(self.module_identifier, params[0])
            self.rtl_daq_socket.send(msg_byte_array)
            reply = self.rtl_daq_socket.recv()
            self.logger.debug(f"Received reply: {reply}")
        elif command == "GAIN":
            try:
                if self.noise_source_state: # The noise source is turned on, we are storing only the gains
                    for m in range(self.M):
                        self.last_gains[m] = self.valid_gains.index(params[m])
                else:
                    for m in range(self.M):
                        self.gains[m] = self.valid_gains.index(params[m])
                    self._change_gains()
            except ValueError:
                self.logger.error("Improper gain value {:d}".format(params[m]))
    
    def _control_noise_source(self, noise_source_state):
        """
            Enables or disables the internal noise of the receiver and set the proper gain values.
            
            Gain and frequency values are fittet to the hardware of the Kerberos SDR v2
            You can change here the preset gain values depending on the calibration frequency here!

            Parameters:
            -----------
            :param: noise_source_state: Requested state of the noise control 
                    (When set to True, the noise source will be enabled)
            :type : noise_source_state: Bool

        """
        if noise_source_state:
            self.logger.info("Set gain values to perform calibration") 
            
            for m in range(self.M):
                self.last_gains[m]=self.gains[m]
                self.gains[m]=self.cal_gain_table[1,np.argmin(abs(self.cal_gain_table[0,:]-self.iq_header.rf_center_freq))]
            self._change_gains()

            self.logger.info("Enable noise source, [{:d}]".format(self.iq_header.cpi_index))
            msg_byte_array = inter_module_messages.pack_msg_noise_source_ctr(self.module_identifier, True)
            self.rtl_daq_socket.send(msg_byte_array)
            reply = self.rtl_daq_socket.recv()
            self.logger.debug(f"Received reply: {reply}")                 
            self.noise_source_state = True # Next state
            self.current_state = "STATE_NOISE_CTR_WAIT"
        else:
            self.logger.info("Disabling noise source [{:d}]".format(self.iq_header.cpi_index))
            msg_byte_array = inter_module_messages.pack_msg_noise_source_ctr(self.module_identifier, False)
            self.rtl_daq_socket.send(msg_byte_array)
            reply = self.rtl_daq_socket.recv()
            self.logger.debug(f"Received reply: {reply}")
            self.noise_source_state = False # Next state

            self.logger.info("Restore gain values after calibration")            
            for m in range(self.M):                
                self.gains[m] = self.last_gains[m]
            self._change_gains()

            self.current_state = "STATE_NOISE_CTR_WAIT"


    def start(self):
        """
            Start the main processing loop
        """
        while True:
            
            #############################################
            #           OBTAIN NEW DATA BLOCK           #  
            #############################################
                        
            # Obtained new data                                    
            active_buff_index = self.in_shmem_iface.wait_buff_free()            
            if active_buff_index < 0 or active_buff_index > 1:
                logging.critical("Failed to acquire iq frame, exiting ..")
                break;          

            buffer = self.in_shmem_iface.buffers[active_buff_index]
            iq_header_bytes = buffer[0:1024].tobytes()
            self.iq_header.decode_header(iq_header_bytes)            
            #self.iq_header.dump_header()

            if self.iq_header.check_sync_word():
                logging.critical("IQ header sync word check failed, exiting..")
                break
                
            # IQ samples are currently not required in this module, hence this section is disabled
            # incoming_payload_size = self.iq_header.cpi_length*self.iq_header.active_ant_chs*2*int(self.iq_header.sample_bit_depth/8)
            # if incoming_payload_size > 0:
            	# iq_samples = buffer[1024:1024 + incoming_payload_size].view(dtype=np.complex64).reshape(self.iq_header.active_ant_chs, self.iq_header.cpi_length) [:,0:self.N_proc] 
                
            self.logger.debug("Type:{:d}, CPI: {:d}, State:{:s}".format(self.iq_header.frame_type, self.iq_header.cpi_index, self.current_state))
            ##############################################
            #  Hardware Controller Finite State Machine  #
            ##############################################
            
            if self.iq_header.frame_type != IQHeader.FRAME_TYPE_DUMMY : # Not Dummy Frame 
                
                # -> Check power levels and gain values from the header
                if self.iq_header.frame_type == IQHeader.FRAME_TYPE_DATA:
                    for m in range(self.M):
                        power = 0 # TODO: Read out from the header
                        self.logger.debug("Channel {:d} power:{:.2f} dB, gain:{:d} [{:d}]".format(m, power, 
                                        self.iq_header.if_gains[m], self.iq_header.cpi_index))                                            

                # -> Chech overdrive per channel
                for m in range(self.M):
                    if(self.iq_header.adc_overdrive_flags & 1<<m):
                        self.logger.warning("Overdrive ch {:d} [{:d}]".format(m, self.iq_header.cpi_index))

                #
                #------------------------------------------>
                #            
                if self.current_state == "STATE_INIT": 
                    # Set initial gain values
                    self._change_gains()

                    # Disable internal noise source
                    msg_byte_array = inter_module_messages.pack_msg_noise_source_ctr(self.module_identifier, False)
                    self.rtl_daq_socket.send(msg_byte_array)
                    reply = self.rtl_daq_socket.recv()
                    self.logger.debug(f"Received reply: {reply}")                    

                    # TODO: Set initial ADPIS values here
                    if any(self.gain_tune_states):
                        self.current_state = "STATE_GAIN_CTR_WAIT" 
                    else:
                        self.current_state = "STATE_IQ_CAL" 
                #
                #------------------------------------------>
                #   
                elif self.current_state == "STATE_GAIN_CTR_WAIT":
                    gain_ctr_ready = True
                    # Check gain states
                    for m in range(self.M):
                        if self.valid_gains[self.gains[m]] != self.iq_header.if_gains[m]:
                            gain_ctr_ready = False
                    if gain_ctr_ready:
                        self.current_state = "STATE_GAIN_TUNE"

                #
                #------------------------------------------>
                #   
                elif self.current_state == "STATE_GAIN_TUNE":
                    # Decrease gain if overdrive is detected
                    if self.iq_header.adc_overdrive_flags:
                        self._tune_gains()
                        self.current_state="STATE_GAIN_CTR_WAIT"
                        self.gain_lock_cntr = 0

                    # Increse gain to drive full scale
                    elif any(self.gain_tune_states):
                        self._tune_gains()
                        self.current_state="STATE_GAIN_CTR_WAIT"
                        self.gain_lock_cntr = 0

                    else: # Gain tuning may finished - check gain lock
                        if self.gain_lock_cntr == self.gain_lock_interval:
                            self.current_state = "STATE_IQ_CAL"
                            self.gain_lock_cntr = 0
                        else:
                            self.gain_lock_cntr +=1

                #
                #------------------------------------------>
                #
                elif self.current_state == "STATE_IQ_CAL":             

                    # --> Noise Source Control 
                    if  self.cal_track_mode == 0 or self.cal_track_mode == 1 or \
                        (self.cal_track_mode == 2 and self.cal_frame_cntr < self.cal_frame_interval):
                        
                        # Enable noise source 
                        if self.iq_header.sync_state < 5 : # Delay synchronizer is not in track or track lock mode
                            if self.iq_header.noise_source_state == 0:
                                self._control_noise_source(noise_source_state=True)
                        else: # Delay synchronizer is in track mode - Disable Noise Source
                            if self.iq_header.noise_source_state == 1: # Has sync, disable noise source if enabled                                    
                               # Check Track lock approval (Used, when the system is calibrated and the antennas are detached)
                               track_lock_approval = True
                               if self.require_track_lock_intervention:
                                   self.track_lock_ctr_fd.seek(0, 0)
                                   ctr_char = self.track_lock_ctr_fd.read(1)
                                   if ctr_char == '1':
                                       logging.info("User has approved track lock state initialization")
                                       self.track_lock_ctr_fd.seek(0, 0)
                                   else:
                                       logging.info("Waiting for track lock init. approval")
                                       track_lock_approval = False                               
                               if track_lock_approval:
                                   self._control_noise_source(noise_source_state=False)
                       
                    # --> Burst calibration frames
                    if self.cal_track_mode == 2: 
                        if self.iq_header.sync_state == 6:
                            self.cal_frame_cntr +=1
                            if self.cal_frame_cntr == self.cal_frame_interval:
                                self.logger.info("Enable noise source burst [{:d}]".format(self.iq_header.cpi_index))
                                self._control_noise_source(noise_source_state=True)
                                
                            elif self.cal_frame_cntr == self.cal_frame_interval+self.cal_frame_burst_size:
                                self.logger.info("Disabling noise source burst [{:d}]".format(self.iq_header.cpi_index))
                                self._control_noise_source(noise_source_state=False)
                                self.cal_frame_cntr=0
                        else: # self.iq_header.sync_state < 6
                            self.cal_frame_cntr = 0
                    
                    # --> External control request handling
                    ctr_request_condition.acquire()                    
                    if len(ctr_request) !=0 :
                        self.logger.info("Control request: {:s}".format(ctr_request[0]))
                        self._handle_control_reqest(ctr_request[0], ctr_request[1:])
                        ctr_request.clear()
                        ctr_request_condition.notify()
                    ctr_request_condition.release()

                #          
                #------------------------------------------>
                #   
                elif self.current_state == "STATE_NOISE_CTR_WAIT":                 
                    if self.noise_source_state == self.iq_header.noise_source_state:
                        gain_ctr_ready = True
                        # Check gain states
                        for m in range(self.M):
                            if self.valid_gains[self.gains[m]] != self.iq_header.if_gains[m]:
                                gain_ctr_ready = False
                        if gain_ctr_ready:
                            self.current_state = "STATE_IQ_CAL"
                        
            self.in_shmem_iface.send_ctr_buff_ready(active_buff_index)
            
                        

class CtrIfaceServer(threading.Thread):
            
    def __init__(self, M):
        """
            Initialize the Ethernet socket based control interface
            Parameters:
            -----------
            :param: M: Number of receiver channels in the system
            :type:  M: int
        """

        self.logger = logging.getLogger(__name__)
        threading.Thread.__init__(self)  

        # Control interface server parameters
        self.ctr_iface_port_no = 5001 # TODO: Set this port number from the ini file
        self.ctr_iface_socket =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ctr_iface_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.ctr_iface_addr = ("", self.ctr_iface_port_no)
        self.M = M
        self.status=True
       
    def run(self):
        """
            Starts the control server service thread
        """

        self.logger.info("Opening control interface server on ip address: {:s} and port: {:d}".format("-", self.ctr_iface_port_no))
        try:
            self.ctr_iface_socket.bind(self.ctr_iface_addr)
            self.ctr_iface_socket.listen(1)
        except socket.error:
            self.logger.error("Unable to open the Hardware configuration interface")
            self.status=False
            return -1

        while(True):
            # Wait for a connection
            self.logger.info("Waiting for new connection")
            connection, client_address = self.ctr_iface_socket.accept()

            try:
                self.logger.info("Conenction established ")

                # Main server loop
                while True:
                    ctr_frame = connection.recv(128)
                    if ctr_frame:
                        exit_flag = self.process_ctr_frame(ctr_frame)
                        if exit_flag:
                            break
                        # Send config success
                        msg_bytes=("FNSD".encode()+bytearray(124))
                        connection.send(msg_bytes)
                    else:
                        self.logger.info("No more data from client, closing connection")
                        break
            finally:
                # Clean up the connection
                connection.close()

    def process_ctr_frame(self, msg_bytes):
        """
            Processes the control interface message and prepares the command parameters
            for further command handling.
            
            The input message is composed as follows:
            Total length: 128 byte
            -----------------------------------
            |4 byte cmd|...124 byte payload...|
            -----------------------------------
            For the detailed description of the valid command please check the corresponding
            documentation.
            Parameters:
            -----------
            :param: msg_bytes: Received command, that has to be processed
            :type : msg_bytes: 128 byte length byte array
        """
        ctr_request_condition.acquire()
        command = msg_bytes[0:4].decode()

        if command == "EXIT":
            ctr_request_condition.release()
            return -1

        elif command == "STHU":
            threshold = unpack('f',msg_bytes[4:8])[0]
            self.logger.info("Received threshold value: {:f}".format(threshold))
            ctr_request.clear()
            ctr_request.append(command)
            ctr_request.append(threshold)            

        elif command == "FREQ":
            frequency = unpack('Q',msg_bytes[4:12])[0]
            self.logger.info("Received frequency value: {:f} Hz".format(frequency))
            ctr_request.clear()
            ctr_request.append(command)
            ctr_request.append(frequency)            

        elif command == "GAIN":
            gains = unpack('I'*self.M, msg_bytes[4:4+4*self.M])
            ctr_request.clear()
            ctr_request.append(command)
            for m in range(self.M):
                self.logger.info("Received gain values - CH{:d}: {:d} dB x 10".format(m, gains[m]))            
                ctr_request.append(gains[m])

        elif command == "INIT":
            self.logger.info("Inititalization command received")
            ctr_request.clear()
            ctr_request.append(command)
        else:
            self.logger.error("Unidentified control command: {:s}".format(command))

        ctr_request_condition.wait() # Let the main thread process the request       
        ctr_request_condition.release()
        return 0

if __name__ == "__main__":
    HWC_inst0 = HWC()
    if HWC_inst0.init() == 0:
        HWC_inst0.start()

HWC_inst0.close()
