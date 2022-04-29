#!/bin/bash
#NOTE: Use for KerberosSDR Only

echo "This software will initialze KerberosSDR EEPROM content and set serial numbers"
device_count=$(lsusb | grep "Realtek" | wc -l)
echo "Found $device_count receivers"
while true; do
    read -p "Do you wish to overwrite the current EEPROM content?" yn
    case $yn in
        [Yy]* )echo "yes"; break;;
        [Nn]* )echo "no"; exit;;
        *) echo "Please answer yes or no"
    esac
done
device_cntr=$((device_count-1))
for i in $(eval echo "{0..$device_cntr}")
do
    dip_sw_count=$((i+1))
    read -p "Please turn off all DIP switches except #$dip_sw_count and press enter" dummy
    # Check the number of online devices
    curr_device_count=$(lsusb | grep "Realtek" | wc -l)
    if test $curr_device_count -ne 1
    then
        echo "More than one online device has been detected, exiting"
        #exit
    fi
    serial=$((i+1000))
    rtl_eeprom -d 0 -g realtek_oem
    rtl_eeprom -d 0 -s $serial -m RTL-SDR -p KerberosSDR
done
echo "EEPROM writing script finished. Plese perform a full power cycle."
