"""
	Description :
	Unit test for the decimator module

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
import warnings
from struct import pack
import time
from configparser import ConfigParser
from scipy import signal

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

import plotly.graph_objects as go 

class TesterDecimatorModule(unittest.TestCase):


    @classmethod
    def setUpClass(cls):        
        logging.basicConfig(filename=join(test_logs_path,'unit_test.log'),
                             level=logging.INFO)   
        logging.info("--> Starting Decimator unit test <--")
        logging.warning("Divide by zero warnings are ignored from numpy")                
        warnings.filterwarnings("ignore", message="numpy.dtype size changed")
        warnings.filterwarnings("ignore", message="numpy.ufunc size changed")
        np.seterr(divide = 'ignore') 
                
        try:
            proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_in")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_in")], stderr=subprocess.DEVNULL)
            proc.wait()

            proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_out")], stderr=subprocess.DEVNULL)
            proc.wait()
            proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_out")], stderr=subprocess.DEVNULL)
            proc.wait()
        except (IOError, OSError):
            pass
        
    def setUp(self):        
                        
        # Open log files        
        self.fd_log_decimator_err = open(join(log_path,"decimator.log"), "w")        
        self.fd_log_gen_err  = open(join(log_path,"gen.log"),  "w")

        # Create fake gate control file
        self.fd_gate_ctr  = open(join(data_control_path,"m_gate_control_fifo"),  "w")
        self.fd_gate_ctr.close()
        
        # Set-up control FIFOs   
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"fw_decimator_in")])
        proc.wait()
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"bw_decimator_in")])
        proc.wait()

        proc = subprocess.Popen(["mkfifo",join(data_control_path,"fw_decimator_out")])
        proc.wait()
        proc = subprocess.Popen(["mkfifo",join(data_control_path,"bw_decimator_out")])
        proc.wait()
                                        
        # Save default config file parameters
        self.N, self. R, self.N_daq, self.N_cal, self.fir_bw, self.K, self.win, self.reset = self._read_config_file()
        
    def tearDown(self):
        # Close log files        
        self.fd_log_decimator_err.close()       
        self.fd_log_gen_err.close()
                        
        # Close control FIFOs        
        proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_in")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_in")], stderr=subprocess.DEVNULL)
        proc.wait()

        proc = subprocess.Popen(["rm",join(data_control_path,"fw_decimator_out")], stderr=subprocess.DEVNULL)
        proc.wait()
        proc = subprocess.Popen(["rm",join(data_control_path,"bw_decimator_out")], stderr=subprocess.DEVNULL)
        proc.wait()
        
        # Remove test data file
        proc = subprocess.Popen(["rm", join(unit_test_path,'decimator_test_0.dat')], stderr=subprocess.DEVNULL)
        proc.wait()        
                
        # Write back default config file parameters
        self._write_config_file(self.N, self. R, self.N_daq, self.N_cal, self.fir_bw, self.K, self.win, self.reset)
    
    #############################################
    #               TEST FUNCTIONS              #
    #############################################    
    #@unittest.skip("Skipped during development")
    def test_case_5_0(self):
        logging.info("-> Starting Test case [5_0]: Data frame forward test")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA
        self._write_config_file(N=2**18, R=1, N_daq=2**18, N_cal=2**10, fir_bw=1.0, K=1, win='hann', reset=0) 
        output_frame_count = frame_count   
        # -> Action <-
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        self._run_std_frame_test(frame_count, frame_type)               
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'decimator_test_0.dat'), output_frame_count, frame_type))
    #@unittest.skip("Skipped during development")
    def test_case_5_1(self):
        logging.info("-> Starting Test case [5_1]:")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DUMMY
        self._write_config_file(N=2**18, R=1, N_daq=2**18, N_cal=2**10, fir_bw=1.0, K=1, win='hann', reset=0)
        output_frame_count = frame_count
        # -> Action <-
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        self._run_std_frame_test(frame_count, frame_type)               
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'decimator_test_0.dat'), output_frame_count, frame_type))
    #@unittest.skip("Skipped during development")
    def test_case_5_2(self):
        logging.info("-> Starting Test case [5_2]:")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_CAL
        self._write_config_file(N=2**18, R=3, N_daq=2**10, N_cal=2**18, fir_bw=1.0, K=1, win='hann', reset=0)
        output_frame_count = frame_count   
        # -> Action <-
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        self._run_std_frame_test(frame_count, frame_type)               
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'decimator_test_0.dat'), output_frame_count, frame_type))    
    #@unittest.skip("Skipped during development")
    def test_case_5_3(self):
        logging.info("-> Starting Test case [5_3]:")
        # -> Assume <-
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_TRIGW
        self._write_config_file(N=2**18, R=7, N_daq=2**14, N_cal=2**10, fir_bw=1.0, K=1, win='hann', reset=0)
        output_frame_count = frame_count   
        # -> Action <-
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        self._run_std_frame_test(frame_count, frame_type)     
        # -> Assert <-        
        self.assertFalse(self.check_frame(join(unit_test_path,'decimator_test_0.dat'), output_frame_count, frame_type))
    #@unittest.skip("Skipped during development")
    def test_case_5_4(self):
        logging.info("-> Starting Test case [5_0]: Data frame forward test with non zero decimation ratio")
        # -> Assume <-
        decimation_ratio = 128
        daq_block_size = 8182
        cpi_size = 8192
        cal_size = 2048
        frame_size = daq_block_size*decimation_ratio
        frame_count = 10
        frame_type  = IQHeader.FRAME_TYPE_DATA
        self._write_config_file(N=cpi_size, R=decimation_ratio, N_daq=daq_block_size, N_cal=cal_size, 
                                fir_bw=1.0, K=2*decimation_ratio, win='hann', reset=0)
        output_frame_count = frame_count
        # -> Action <-
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        self._run_std_frame_test(frame_count, frame_type, frame_size)
        # -> Assert <-
        self.assertFalse(self.check_frame(join(unit_test_path,'decimator_test_0.dat'),output_frame_count, frame_type))
    @unittest.skip("Skipped during development")
    def test_case_5_10(self):
        logging.info("-> Starting Test Case [5_11] : Transfer function test")
        # -> Assume <-        
        decimation_ratio = 383
        N = 2**10
        N_cal = 2**13
        fir_bw = 0.9
        K = 500
        win='hann'
        self._write_config_file(N=N, R=decimation_ratio, N_daq=N, N_cal=N_cal, fir_bw=fir_bw, K=K, win=win, reset=0)
        # -> Action <-
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        self._run_swept_cw_test(decimation_ratio, sample_size=N)
        # -> Assert <-        
        transfer_pass = self.check_transfer(join(unit_test_path,'decimator_test_0.dat'), decimation_ratio, fir_bw, K, win)
        self.assertFalse(np.array(transfer_pass).all())
    @unittest.skip("Skipped during development")
    def test_case_5_11(self):
        logging.info("-> Starting Test Case [5_11] : Phase continuity test")
        # -> Assume <-        
        keep_out  = 5 # Number of samples skipped at the begining due to transient effect
        tolerance = 0.05 # Allowed phase jump in radian
        decimation_ratio = 5  
        N = 2**7
        N_cal = 2**7
        self._write_config_file(N=N, R=decimation_ratio, N_daq=N, N_cal=N_cal, fir_bw=1.0, K=2*decimation_ratio, win='hann', reset=0)
        # -> Action <-
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        self._run_swept_cw_test(decimation_ratio, source_type="cw", sample_size=N)
        # -> Assert <-      
        phase_diff_check = self.check_phase_continuity(join(unit_test_path,'decimator_test_0.dat'), decimation_ratio, tolerance, keep_out)  
        self.assertFalse(phase_diff_check)
    #@unittest.skip("Skipped during development")
    def test_case_5_20(self):        
        logging.info("-> Starting Test case [5_20]: Ramp test on data type frames")
        # -> Assume <-
        frame_type  = IQHeader.FRAME_TYPE_DATA
        frame_count = 100
        N = 2**18
        N_cal = 2**10
        self._write_config_file(N=N, R=1, N_daq=N, N_cal=N_cal, fir_bw=1.0, K=1, win='hann', reset=0)
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        # -> Action
        # Start the frame generator
        generator = rampFrameGenator(frame_type, frame_count, N, 'CINT8', 'decimator_in')
        generator.start()
        
        # Start the decimator module
        decimator_module = subprocess.Popen([join(daq_core_path,"decimate.out"),'0'],                                             
                                    stdout=subprocess.DEVNULL,
                                    stderr=self.fd_log_decimator_err)        
        recorder = IQFrameRecorder("decimator_out",
                                   join(unit_test_path,'decimator_test_0.dat'),
                                   "1")
        if recorder.in_shmem_iface.init_ok:
            recorder.start()
        else:
            self.assertTrue(0)        
        generator.join()
        recorder.join()
        logging.info("Ramp test finished, checking output..")        
        self.assertFalse(self.check_ramp(join(unit_test_path,'decimator_test_0.dat'), frame_count))
    #@unittest.skip("Skipped during development")
    def test_case_5_21(self):        
        logging.info("-> Starting Test case [5_21]: Ramp test on calibration type frames")
        # -> Assume <-
        frame_type  = IQHeader.FRAME_TYPE_CAL
        frame_count = 100
        N_cpi = 7919 # Prime number, should be irrelevent in this test
        N_daq = 2**10
        N_cal = 2**10
        self._write_config_file(N=N_cpi, R=3, N_daq=N_daq, N_cal=N_cal, fir_bw=1.0, K=1, win='hann', reset=0)
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        # -> Action
        # Start the frame generator
        generator = rampFrameGenator(frame_type, frame_count, N_cal, 'CINT8', 'decimator_in')
        generator.start()
        
        # Start the decimator module
        decimator_module = subprocess.Popen([join(daq_core_path,"decimate.out"),'0'],                                             
                                    stdout=subprocess.DEVNULL,
                                    stderr=self.fd_log_decimator_err)        
        recorder = IQFrameRecorder("decimator_out",
                                   join(unit_test_path,'decimator_test_0.dat'),
                                   "1")
        if recorder.in_shmem_iface.init_ok:
            recorder.start()
        else:
            self.assertTrue(0)        
        generator.join()
        recorder.join()
        logging.info("Ramp test finished, checking output..")        
        self.assertFalse(self.check_ramp(join(unit_test_path,'decimator_test_0.dat'), frame_count))
    @unittest.skip("Skipped during development")
    def test_case_5_100(self):
        logging.info("-> Starting Test Case [5_100] : Throughput testing")
        
        # -> Assume <-
        frame_count = 100 
        decimation_ratio = 5
        tap_size = 20
        N = 2**18
        self._write_config_file(N=N, R=decimation_ratio, N_daq=N, N_cal=N, fir_bw=1.0, K=tap_size, win='hann', reset=0)            
        
        # -> Action
        proc = subprocess.Popen(["./fir_filter_designer.py",], stdout=subprocess.DEVNULL)
        proc.wait()
        data_throughput_ratio = self._run_throughput_test(frame_count, decimation_ratio, N)
                
        # -> Assert <-
        self.assertTrue(data_throughput_ratio > 1)

    #############################################
    #              TEST RUN WRAPPERS            #  
    #############################################
    
    def _run_std_frame_test(self,frame_count, frame_type, sample_size=2**18):        
        # Build up and start test chain
        generator = stdFrameGenator(frame_type, frame_count, sample_size, 'CINT8', 'decimator_in')
        generator.start()
 
        decimator_module = subprocess.Popen([join(daq_core_path,"decimate.out"),'0'],                                             
                                            stdout=subprocess.DEVNULL,
                                            stderr=self.fd_log_decimator_err)
        recorder = IQFrameRecorder("decimator_out",
                                   join(unit_test_path,'decimator_test_0.dat'),
                                   "1")
        if recorder.in_shmem_iface.init_ok:
            recorder.start()
        else:
            self.assertTrue(0)        
        decimator_module.wait()    
        generator.join()
        recorder.join()  
        
    def _run_swept_cw_test(self, decimation_ratio=1, source_type="swept-cw", sample_size=2**18):
        # Build up and start test chain
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_cw.py"),
                                      '-r',str(decimation_ratio),
                                      '-s', source_type,
                                      '-n',str(sample_size),
                                      "-m", "decimator_in"], 
                                     stdout=subprocess.DEVNULL, 
                                     stderr=self.fd_log_gen_err)
        decimator_module = subprocess.Popen([join(daq_core_path,"decimate.out"),'0'],                                              
                                            stdout=subprocess.DEVNULL,
                                            stderr=self.fd_log_decimator_err)
        recorder = IQFrameRecorder("decimator_out",
                                   join(unit_test_path,'decimator_test_0.dat'),
                                   "1")
        if recorder.in_shmem_iface.init_ok:
            recorder.start()
        else:
            self.assertTrue(0)
        generator.wait()        
        decimator_module.wait()
        recorder.join()
        
    def _run_throughput_test(self, frame_count, decimation_ratio=1, sample_size=2**18):
        # Start frame generator
        generator = subprocess.Popen(["python3",join(unit_test_path,"gen_std_frame.py"),                                
                                "-b",str(frame_count),
                                "-n", str(sample_size),
                                "-m", "decimator_in"], 
                                stdout=subprocess.DEVNULL, 
                                stderr=self.fd_log_gen_err)
        # Start decimator module        
        decimator_module = subprocess.Popen([join(daq_core_path,"decimate.out"),'0'],                                             
                                            stdout=subprocess.DEVNULL,
                                            stderr=self.fd_log_decimator_err)   
        recorder = IQFrameRecorder("decimator_out",
                                   join(unit_test_path,'decimator_test_0.dat'),
                                   "0")        
        if recorder.in_shmem_iface.init_ok:
            recorder.start()
        else:            
            self.assertTrue(0)
        decimator_module.wait()
        recorder.join()        
                
        # -> Evaluation
        data_throughput  = (frame_count)*sample_size/10**6/recorder.elapsed_time
        data_throughput_ratio = data_throughput/2.4
        frame_throughput = (frame_count)/recorder.elapsed_time
        nominal_frame_throughput = 1/(sample_size/(2.4*10**6))

        logging.info("Decimator module total runtime: {:.2f}".format(recorder.elapsed_time))
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
                    #iq_header.dump_header()                                                   
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
        
    def check_transfer(self, file_name, decimation_ratio, fir_bw, K, win):
        iq_header = IQHeader()
        frequencies = []
        transfers   = []        
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
                -------------------------
                  Extract trasnfer value
                -------------------------
                """
                # Check payload content
                if data_valid:
                    current_freq=iq_header.rf_center_freq/10**6-100
                    fs = iq_header.sampling_freq/10**6
                    #if current_freq !=0 :
                    frequencies.append(current_freq)
                    iq_cf64 = np.frombuffer(iq_data_bytes, dtype=np.complex64).reshape(iq_header.active_ant_chs, iq_header.cpi_length)
                    iq_cf64 = iq_cf64.copy()                         
                    for m in range(iq_header.active_ant_chs):
                        # DC offset compensation - Use only with max peak search
                        # iq_cf64[m, :]-=-np.average(iq_cf64[m, :]
                        xw = np.fft.fft(iq_cf64[m, :])
                        xw = np.fft.fftshift(xw)
                        xw = abs(xw)
                        
                        max_index = np.argmax(xw)
                        
                        aliased_freq = current_freq-1*fs*round(current_freq/fs)
                        freq_bin = ((aliased_freq/fs)* len(xw)+len(xw)/2)
                        freq_bin = round(freq_bin)
                        if freq_bin == len(xw): freq_bin=0                        
                                                
                        #transfers.append(xw[max_index]/iq_header.cpi_length) # Extract transfer value with max peak search
                        transfers.append(xw[freq_bin]/iq_header.cpi_length) # Extract transfer value with frequency bin indexing
        """
        -------------------------------
            Compare transfer functions
        -------------------------------
        """        
        transfer_pass=[]        
        freq_points = len(frequencies)
        excitation_signal_amplitude = -3 # dB
        tolerance = 0.5 # dB
        min_transfer_value = -45
        logging.info("Measurement frequency points: {:d}".format(freq_points))

        # Generate reference filter response
        
        cut_off = fir_bw/decimation_ratio
        if cut_off < 1:
            # Design band pass FIR Filter
            filter_coeffs = signal.firwin(K, cut_off, window=win)
        else:
            filter_coeffs=np.array([1])

        #filter_coeffs = np.ones(decimation_ratio)
        #filter_coeffs /= np.sum(np.abs(filter_coeffs)) 
        w, h = signal.freqz(b=filter_coeffs, a=1, worN=freq_points, whole=True)
        h = np.abs(h)
        h = np.fft.fftshift(h)
        h_log = 20*np.log10(h)+excitation_signal_amplitude

        # Prepare figure to display responses        
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=frequencies, y=h_log,  name= "Reference", line=dict(width=2, dash='dash')))

        # Compare obtained responses with the reference
        for m in range(iq_header.active_ant_chs):
            # -> Calulcation
            trasnfer_ch_m = transfers[m::iq_header.active_ant_chs]
            meas_transfer_log = 20*np.log10(trasnfer_ch_m)
            transfer_diff = h_log- meas_transfer_log
            transfer_diff [h_log < min_transfer_value] = 0
            transfer_diff = abs(transfer_diff)
            tolerance_check = (transfer_diff > tolerance).any()
            transfer_pass.append(tolerance_check)
            
            # -> Logging
            logging.info('Checking transfer on channel: {:d}'.format(m))            
            logging.info("Maximum deviation: {:.2f} dB".format(max(transfer_diff)))            
            fig.add_trace(go.Scatter(x=frequencies,y=meas_transfer_log, name = "Channel {:d}".format(m), line=dict(width=2, dash='solid')))
            fig.add_trace(go.Scatter(x=frequencies,y=transfer_diff, mode='lines+markers', name = "Channel {:d} deviation".format(m), line=dict(width=2, dash='solid')))

        # Export figure
        fig.update_layout(
            title = "Test Case: 5_10 : Decimator Transfer",
            xaxis_title = "Frequency [MHz]",
            yaxis_title = "Normalized amplitude [dB]",
            font=dict(size=18),
            hovermode='x')
        fig.write_html(join(test_logs_path,'TestCase-5_10.html'))
        
        #np.save("transfers.npy",transfers)
        #np.save("frequencies.npy",frequencies)

        return transfer_pass
    def check_phase_continuity(self, file_name, decimation_ratio, tolerance, keep_out):
        iq_header = IQHeader()
        signal_array=None
        blocks = 0
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
                    #iq_header.dump_header() # For debug purposes
                    iq_data_length = int((iq_header.cpi_length * iq_header.active_ant_chs * (2*iq_header.sample_bit_depth))/8)
                    # Reading Multichannel IQ data
                    if iq_data_length > 0:
                        iq_data_bytes = file_descr.read(iq_data_length)                    
                        if len(iq_data_bytes) == 0: break
                        data_valid = True
                        blocks +=1
                except EOFError:
                    pass                    
                """
                -------------------------
                    Accumulate signal
                -------------------------
                """
                # Check payload content
                if data_valid:
                    iq_cf64 = np.frombuffer(iq_data_bytes, dtype=np.complex64).reshape(iq_header.active_ant_chs, iq_header.cpi_length)
                    iq_cf64 = iq_cf64.copy()    
                    # Note: This is quite inefficient but simple and safe
                    if blocks ==1:
                        signal_array = iq_cf64
                    else: 
                        signal_array = np.append(signal_array, iq_cf64,axis=1)  
        
        logging.info("Received blocks: {:d}".format(blocks))       
        signal_array = signal_array[:,keep_out:]    
        """
        -------------------------------
            Check phase continuity
        -------------------------------
        """                
        fs = 2.4*10**6
        f  = fs/256
        
        # Generate reference signal
        amplitude = decimation_ratio
        ref = amplitude*np.exp(1j*(2*np.pi*f/(fs/decimation_ratio)*np.arange(0,len(signal_array[0,:]),1)))

        # Prepare figure to display responses        
        fig = go.Figure()
        #fig.add_trace(go.Scatter(y=ref.real,  name= "Reference-real", line=dict(width=2, dash='dash')))            
        #fig.add_trace(go.Scatter(y=ref.imag,  name= "Reference-imag", line=dict(width=2, dash='dash')))            

        # Compare obtained responses with the reference
        for m in range(iq_header.active_ant_chs):
            # -> Calulcation
            mf_output  = signal_array[m,:] * ref.conj()
            phase_diff = np.abs(np.diff(np.angle(mf_output)))
                                 
            # -> Logging
            logging.info('Checking phase conitnuity on channel: {:d}'.format(m))            
            logging.info("Maximum deviation: {:.2f} rad".format(max(np.abs(phase_diff))))            
            fig.add_trace(go.Scatter(y=signal_array[m,:].real, name = "Channel {:d} - real".format(m), line=dict(width=2, dash='solid')))
            #fig.add_trace(go.Scatter(y=signal_array[m,:].imag, name = "Channel {:d} - imag".format(m), line=dict(width=2, dash='solid')))
            fig.add_trace(go.Scatter(y=np.append(np.zeros(1),phase_diff), name = "Phase diff, channel :{:d}".format(m), line=dict(width=2, dash='solid')))
            
        # Export figure
        fig.update_layout(
            title = "Test Case: 5_11 : Decimator Phase Conitnuity",
            xaxis_title = "Time index [sample]",
            yaxis_title = "Phase difference",
            font=dict(size=18),
            hovermode='x')
        fig.write_html(join(test_logs_path,'TestCase-5_11.html'))       
        
        return (phase_diff > tolerance).any() 
    
    def check_ramp(self, file_name, frame_count):
        iq_header = IQHeader()
        ramp_max = 29
        DC = 127.5
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
                    #iq_data = np.frombuffer(iq_data_bytes, dtype=np.uint8).reshape(iq_header.active_ant_chs, iq_header.cpi_length*2)            
                    iq_cf64 = np.frombuffer(iq_data_bytes, dtype=np.complex64).reshape(iq_header.active_ant_chs, iq_header.cpi_length)
                    iq_cf64 = iq_cf64.copy()

                    # Revert normalization and DC compensation made in the decimator
                    iq_cf64 = iq_cf64*DC + DC*(1+1j)

                    raw_sig = np.arange(time_index, time_index + iq_header.cpi_length,1, dtype=np.uint32)%ramp_max
                    for m in range(iq_header.active_ant_chs):
                        ch_m_data_i = iq_cf64[m,:].real
                        ch_m_data_q = iq_cf64[m,:].imag
                        diff_i = (raw_sig[:]+2*m*ramp_max - ch_m_data_i)
                        diff_q = (raw_sig[:]+(2*m+1)*ramp_max - ch_m_data_q)
                        if (diff_i < 0.1).all() and (diff_q < 0.1).all(): 
                            pass
                            logging.info("Block {:d}, Channel {:d} - Passed".format(iq_header.daq_block_index, m))                    
                        else:
                            errors = 0
                            logging.error("Ramp test failed!")
                            logging.error("DAQ block index: {:d}".format(iq_header.daq_block_index))
                            logging.error("Channel: {:d}".format(m))
                            logging.error("Expected <-> Received")              
                            for n in range(iq_header.cpi_length):                                
                                if diff_i[n] > 0.1 or diff_q[n] > 0.1:
                                    errors +=1
                                    #logging.error("Diff I: {:f}, Diff Q: {:f}".format(diff_i[n], diff_q[n]))
                                    logging.error("CH I ind:{:d}, [{:d}<->{:f}]"\
                                        .format(n,(raw_sig[:]+2*m*ramp_max)[n],ch_m_data_i[n]))
                                    logging.error("CH Q ind:{:d}, [{:d}<->{:f}]"\
                                        .format(n,(raw_sig[:]+(2*m+1)*ramp_max)[n],ch_m_data_q[n]))      
                                if errors > 10:
                                    break

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
            return 0,0,0,0,0,'',0
        N       = parser.getint('pre_processing', 'cpi_size')        
        R       = parser.getint('pre_processing', 'decimation_ratio')        
        N_daq   = parser.getint('daq','daq_buffer_size')
        fir_bw  = parser.getfloat('pre_processing', 'fir_relative_bandwidth')
        K       = parser.getint('pre_processing', 'fir_tap_size')
        win     = parser.get('pre_processing','fir_window')
        reset   = parser.getint('pre_processing','en_filter_reset')
        N_cal   = parser.getint('calibration', 'corr_size')
        
        return (N, R, N_daq, N_cal, fir_bw, K, win, reset)
    
    def _write_config_file(self, N, R, N_daq, N_cal, fir_bw, K, win, reset):
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
        parser['pre_processing']['fir_relative_bandwidth'] = str(fir_bw)
        parser['pre_processing']['fir_tap_size'] = str(K)
        parser['pre_processing']['fir_window'] = win
        parser['pre_processing']['en_filter_reset'] = str(reset) 
        with open(config_filename, 'w') as configfile:
            parser.write(configfile)
        return 0
