import numpy as np
import matplotlib.pyplot as plt
import os
import logging 
import sys
# Import IQ header module
currentPath = os.path.dirname(os.path.realpath(__file__))
rootPath = os.path.dirname(currentPath)
sys.path.insert(0, os.path.join(rootPath, "_daq_core"))
from iq_header import IQHeader


logging.basicConfig(level=logging.INFO)
"""
---------------------
     P A R A M S
---------------------    
"""

file_name = "VEGAM_2.iqf"
file_descr = open(file_name, "rb")
fs = 2.4*10**6
std_ch_ind = 0

en_td_plot    = True
en_fd_plot    = False
en_xcorr_plot = False
en_iq_diff_plot = True

td_plot_ch_ind = 0
xcorr_channels = [0,1]
"""
---------------------
       L O A D
---------------------    
"""

iq_header_bytes = file_descr.read(1024)
iq_header = IQHeader()
iq_header.decode_header(iq_header_bytes)
iq_header.dump_header()

iq_data_length = int((iq_header.cpi_length * iq_header.active_ant_chs * (2*iq_header.sample_bit_depth))/8)
iq_data_bytes = file_descr.read(iq_data_length)


file_descr.close()
          
iq_cf64 = np.frombuffer(iq_data_bytes, dtype=np.complex64).reshape(iq_header.active_ant_chs, iq_header.cpi_length)
iq_cf64 = iq_cf64.copy()

"""
---------------------
      P L O T S
---------------------    
"""
N = iq_header.cpi_length
M = iq_header.active_ant_chs

# Remove DC
for m in range(M):       
    iq_cf64[m,:] -= np.average(iq_cf64[m,:])

if en_td_plot:
    x = iq_cf64[0,:]
    plt.figure(1)
    #plt.plot(x.real[0:200])
    #plt.plot(x.imag[0:200])
    plt.plot(iq_cf64[td_plot_ch_ind,0:200].real)
    plt.plot(iq_cf64[td_plot_ch_ind,0:200].imag)

if en_fd_plot:
    plt.figure(2)
    freqs = np.fft.fftfreq(N, 1/fs)
    freqs = np.fft.fftshift(freqs) 
    freqs /= 10**6
    for m in range(1):
        xw = np.fft.fft(iq_cf64[m, :])
        xw = np.fft.fftshift(xw)
        xw = abs(xw)
        xw /= np.max(xw)
        plt.plot(freqs, 20*np.log10(xw))

if en_xcorr_plot:
    plt.figure(3)
    
    N_proc = 2**16
    np_zeros = np.zeros(N_proc, dtype=np.complex64)
    x_padd = np.concatenate([iq_cf64[std_ch_ind, 0:N_proc], np_zeros])
    x_fft = np.fft.fft(x_padd)
    
    time_delay_indices = np.arange(0,2*N_proc)-N_proc
    for m in np.arange(1, M,1):        
        y_padd = np.concatenate([np_zeros, iq_cf64[m, 0:N_proc]])
        y_fft = np.fft.fft(y_padd)
        corr_function = np.fft.ifft(x_fft.conj() * y_fft)
        corr_function = abs(corr_function)
        corr_function_log = 20*np.log10(corr_function)
        #corr_function_log -= max(corr_function_log) 
        
        plt.plot(time_delay_indices,corr_function_log)
        
    plt.xlim([-100, 100])
    #plt.ylim([-60,0])
    plt.xlabel("Time delay [sample]")
    plt.ylabel("Amplitude [dB]")
    
if en_iq_diff_plot:
    plt.figure(4)    
    xcorr = iq_cf64[xcorr_channels[0],:] * np.conjugate(iq_cf64[xcorr_channels[1],:])
    xcorr /= np.max(np.abs(xcorr))
    plt.scatter(xcorr.real, xcorr.imag)