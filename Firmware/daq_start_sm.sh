#!/bin/bash
#
#   DAQ chain start srcipt
#
#   Project : HeIMDALL DAQ Firmware
#   License : GNU GPL V3
#   Authors: Tamas Peto, Carl Laufer

# Check config file
#res=$(python3 ini_checker.py 2>&1) #comment out ini checker for now since it is very slow
echo -e "\e[33mConfig file check bypassed [ WARNING ]\e[39m"
#if test -z "$res" 
#then
#      echo -e "\e[92mConfig file check [ OK ]\e[39m"
#else
#      echo -e "\e[91mConfig file check [ FAIL ]\e[39m"
#      echo $res
#      exit
#fi

sudo sysctl -w kernel.sched_rt_runtime_us=-1

# Read config ini file
out_data_iface_type=$(awk -F "=" '/out_data_iface_type/ {print $2}' daq_chain_config.ini)

# (re) create control FIFOs
rm _data_control/fw_decimator_in 2> /dev/null
rm _data_control/bw_decimator_in 2> /dev/null

rm _data_control/fw_decimator_out 2> /dev/null
rm _data_control/bw_decimator_out 2> /dev/null

rm _data_control/fw_delay_sync_iq 2> /dev/null
rm _data_control/bw_delay_sync_iq 2> /dev/null

rm _data_control/fw_delay_sync_hwc 2> /dev/null
rm _data_control/bw_delay_sync_hwc 2> /dev/null

mkfifo _data_control/fw_decimator_in
mkfifo _data_control/bw_decimator_in

mkfifo _data_control/fw_decimator_out
mkfifo _data_control/bw_decimator_out

mkfifo _data_control/fw_delay_sync_iq
mkfifo _data_control/bw_delay_sync_iq

mkfifo _data_control/fw_delay_sync_hwc
mkfifo _data_control/bw_delay_sync_hwc

# Remove old log files
rm _logs/*.log 2> /dev/null

# Useful to set this on low power ARM devices 
#sudo cpufreq-set -g performance

# Set for Tinkerboard with heatsink/fan
#sudo cpufreq-set -d 1.8GHz

# The Kernel limits the maximum size of all buffers that libusb can allocate to 16MB by default.
# In order to disable the limit, you have to run the following command as root:
sudo sh -c "echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb"

# This command clear the caches
echo '3' | sudo tee /proc/sys/vm/drop_caches > /dev/null

# Check ports(IQ server:5000, Hardware controller:5001)
while true; do
    port_ready=1
    lsof -i:5000 >/dev/null
    out=$?
    if test $out -ne 1
    then
        port_ready=0
    fi
    lsof -i:5001 >/dev/null
    out=$?
    if test $out -ne 1
    then
        port_ready=0
    fi
    if test $port_ready -eq 1
    then
        break
    else
        echo "WARN:Ports used by the DAQ chain are not free! (5000 & 5001)"
        ./daq_stop.sh
        sleep 1
    fi
done

# Generating FIR filter coefficients
python3 fir_filter_designer.py
out=$?
if test $out -ne 0
    then
        echo -e "\e[91mDAQ chain not started!\e[39m"
        exit
fi

# Start main program chain -Thread 0 Normal (non squelch mode)
echo "Starting DAQ Subsystem"
chrt -f 99 _daq_core/rtl_daq.out 2> _logs/rtl_daq.log | \
chrt -f 99 _daq_core/rebuffer.out 0 2> _logs/rebuffer.log &

# Decimator - Thread 1
chrt -f 99 _daq_core/decimate.out 2> _logs/decimator.log &

# Delay synchronizer - Thread 2
chrt -f 99 python3 _daq_core/delay_sync.py 2> _logs/delay_sync.log &

# Hardware Controller data path - Thread 3
chrt -f 99 sudo env "PATH=$PATH" python3 _daq_core/hw_controller.py 2> _logs/hwc.log &
# root priviliges are needed to drive the i2c master

if [ $out_data_iface_type = eth ]; then
    echo "Output data interface: IQ ethernet server"
    chrt -f 99 _daq_core/iq_server.out 2>_logs/iq_server.log &
elif [ $out_data_iface_type = shmem ]; then
    echo "Output data interface: Shared memory"
fi

# IQ Eth sink used for testing
#sleep 3
#python3 _daq_core/iq_eth_sink.py 2>_logs/iq_eth_sink.log &

echo -e "      )  (     "
echo -e "      (   ) )  "
echo -e "       ) ( (   "
echo -e "     _______)_ "
echo -e "  .-'---------|" 
echo -e " (  |/\/\/\/\/|"
echo -e "  '-./\/\/\/\/|"
echo -e "    '_________'"
echo -e "     '-------' "
echo -e "               "
echo -e "Have a coffee watch radar"
