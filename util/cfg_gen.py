#!/usr/bin/env python3
"""
	This script generates config file for the DAQ chain based on the given signal parameters.
    The signal is assumed to be burst like with the following parameters:
        - burst repetition interval [ms]
        - burst length [ms]
        - signal bandwidth [kHz] 
	
	Project: HeIMDALL DAQ Firmware
	Author : Tamas Peto	
"""
import numpy as np
import logging
from configparser import ConfigParser
import argparse
import sys, os

current_path        = os.path.dirname(os.path.realpath(__file__))
firmware_path       = os.path.join(os.path.dirname(current_path),"Firmware")
sys.path.insert(0, firmware_path)

import ini_checker

"""

Prepare config file template

"""
#[meta]
meta={"ini_version":"2",
      "config_name":"autogenV1"
    }
#[hw]
hw={"name"    :"k5",
    "unit_id" :"0",
    "ioo_type":"0",
    "num_ch"  :"5"
}
#[daq]
daq = {
    "log_level"             :"3",
    "daq_buffer_size"       :"262144",
    "center_freq"           :"600000000",
    "sample_rate"           :"1000000",
    "gain"                  :"0",
    "en_noise_source_ctr"   :"1",
    "ctr_channel_serial_no" :"1004"
}
#[squelch]
squelch = {
    "en_squelch"         :"1",
    "amplitude_threshold":"0.5"
}
#[pre_processing]
pre_processing = {
    "cpi_size"               :"262144",
    "decimation_ratio"       :"1",
    "fir_relative_bandwidth" :"1",
    "fir_tap_size"           :"1",
    "fir_window"             :"hann",
    "en_filter_reset"        :"0"
}

#[calibration]
calibration = {
    "corr_size"                       :"65536",
    "std_ch_ind"                      :"0",
    "en_frac_cal"                     :"0",
    "en_iq_cal"                       :"1",
    "amplitude_cal_mode"              :"channel_power",
    "gain_lock_interval"              :"20",
    "unified_gain_control"            :"0",
    "require_track_lock_intervention" :"0",
    "cal_track_mode"                  :"2",
    "cal_frame_interval"              :"100",
    "cal_frame_burst_size"            :"30",
    "amplitude_tolerance"             :"1",
    "phase_tolerance"                 :"2",
    "maximum_sync_fails"              :"5",
}
#[adpis]
adpis = {
    "en_adpis"         :"0",
    "en_gain_tune_init":"0",
    "adpis_proc_size"  :"8192",
    "adpis_gains_init" :"0,0,0,0,0",
}
#[data_interface]
data_interface = {
    "out_data_iface_type" : "shmem"
}

daq_chain_ini_cfg = {"meta"           : meta,
                     "hw"             : hw,
                     "daq"            : daq,
                     "squelch"        : squelch,
                     "pre_processing" : pre_processing,
                     "calibration"    : calibration,
                     "adpis"          : adpis,
                     "data_interface" : data_interface
                     }


"""

Calculate signal specific parameters

"""
parser = argparse.ArgumentParser(description='Automatic configuration file generation for burst type signals')

parser.add_argument('--bri', type=float, nargs=1, required=True,
                     help='Burst repetition interval in miliseconds [ms]')

parser.add_argument('--burst_length', type=float, nargs=1, required=True,
                     help='Burst length in miliseconds [ms]')

parser.add_argument('--bw', type=float, nargs=1, required=True,
                     help='Signal bandwidth in kilohertz [kHz]')

args =vars( parser.parse_args())

# ----------------> MANDATORY PARAMETERS <----------------
BRI          = args['bri'][0]         # burst repetition interval [ms]
burst_length = args['burst_length'][0]# [ms]
bw           = args['bw'][0]          # signal bandwidth [kHz] 
# ---------------->  Extracted from CLI  <----------------

minimum_FIR_tap_size = 16
logging.info("Preparing config file..")
cfg_fname="autogen.ini"

fs = 2400000 # Set sampling rate to maximum to be able to increase the ENOBs as much as possible
decimation_ratio = int(fs/(bw*10**3))
fir_tap_size = 2**(int(np.log2(decimation_ratio))+1) 
if fir_tap_size < minimum_FIR_tap_size: fir_tap_size=minimum_FIR_tap_size
fs_dec     = fs/decimation_ratio # Decimated sample rate
cpi_size   = int(burst_length*10**-3 * fs_dec)

corr_size = 2**int(np.log2(cpi_size))

daq_chain_ini_cfg['daq']['sample_rate'] = str(fs)
daq_chain_ini_cfg['pre_processing']['decimation_ratio'] = str(decimation_ratio)
daq_chain_ini_cfg['pre_processing']['fir_tap_size'] = str(fir_tap_size)
daq_chain_ini_cfg['pre_processing']['cpi_size'] = str(cpi_size)

daq_chain_ini_cfg['calibration']['corr_size'] = str(corr_size)

logging.info("Decimation ratio :{:d}".format(decimation_ratio))
logging.info("FIR filter tap size :{:d}".format(fir_tap_size))
logging.info("CPI length :{:.2f} ms - {:d} sample".format(cpi_size*decimation_ratio/fs*10**3, cpi_size))
"""

Check and write config file

"""

error_list = ini_checker.check_ini(daq_chain_ini_cfg, en_hw_check=False)
if len(error_list):
    for e in error_list:
        logging.error(e)
    logging.critical("Config file generation failed")

else:
    parser = ConfigParser()
    parser.read_dict(daq_chain_ini_cfg)
    with open(cfg_fname, 'w') as configfile:
        parser.write(configfile)
    logging.info("Config file writen to :{0}".format(cfg_fname))
