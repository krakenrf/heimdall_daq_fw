#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
	Designs an FIR filter for the decimator module based on specified parameters found
	in the "daq_chain_config.ini" file
	
	For the proper FIR configuration please check the documentation of the firmware.

	After starting the daq chain you can check the transfer characteristics of the generated
	FIR filter at : "_logs/Decimator_filter_transfer.html"
	
	Project: HeIMDALL DAQ Firmware
	Author : Tamas Peto	
"""
import sys
from scipy import signal
import numpy as np
#import plotly.graph_objects as go 
from configparser import ConfigParser

parser = ConfigParser()
found = parser.read(["daq_chain_config.ini"])
if not found:
	print("Configuration file not found. exiting")
	exit(-1)
decimation_ratio = parser.getint('pre_processing', 'decimation_ratio')
bandwidth = parser.getfloat('pre_processing', 'fir_relative_bandwidth')
tap_size = parser.getint('pre_processing', 'fir_tap_size')
window = parser.get('pre_processing', 'fir_window')
fs = parser.getfloat('daq', 'sample_rate')
fc = parser.getfloat('daq','center_freq')

if decimation_ratio <1:
	print("ERROR: Decimation ratio can not be smaller than 1, exiting..")
	exit(-1)

if tap_size <= decimation_ratio and decimation_ratio !=1 :
	print("ERROR: FIR tap size must be higher than the decimation ratio. Please consider increasing the tap size, exiting..")
	exit(-1)

print("Desig FIR filter with the following parameters: ")
print("Decimation ratio: {:d}".format(decimation_ratio))
print("Bandwidth: {:1.2f}".format(bandwidth))
print("Tap size: {:d}".format(tap_size))
print("Window function:", window)

transfer_fname = "_logs/Decimator_filter_transfer.html"
coeffs_fname = "_data_control/fir_coeffs.txt"
# Fir filter parameters
cut_off = bandwidth/decimation_ratio
if cut_off < 1:
	# Design band pass FIR Filter
	b = signal.firwin(tap_size, cut_off, window=window)
else:
	b=np.array([1])

# Uncomment for debugging
"""
# Plot transfer function
w, h = signal.freqz(b=b, a=1, worN=2**16, whole=True)
h+= 10**-10 # To avoid operations with zero
h =np.fft.fftshift(h)
h_log = 20*np.log10(np.abs(h))

w= np.linspace(-fs/2,  fs/2, 2**16)+fc
fig = go.Figure()
fig.add_trace(go.Scatter(x=w/10**6, y=h_log,  line=dict(width=2, dash='solid')))   

# Export figure
fig.update_layout(
            title = "Decimator filter transfer function",
            xaxis_title = "Frequency [kHz]",
            yaxis_title = "Amplitude[dB]",
            font=dict(size=18),
            hovermode='x')

fig.write_html(transfer_fname)
"""
np.savetxt(coeffs_fname, b)
print("FIR filter ready")
print("Transfer funcfion is exported to : ", transfer_fname)
print("Coefficients are exported to: ",coeffs_fname)
exit(0)
