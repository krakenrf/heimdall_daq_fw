#!/bin/bash
echo "Installing dependencies and build HeIMDALL DAQ Firmware"
cd ..
cd ..
sudo apt install git
echo "6/1 Install build dependencies for the realtek driver"
sudo apt install cmake
sudo apt install libusb-1.0-0-dev
echo "6/2 Build and install rtl-sdr driver"
git clone https://github.com/krakenrf/librtlsdr

cd librtlsdr
mkdir build
cd build
cmake ../ -DINSTALL_UDEV_RULES=ON
make
sudo make install
sudo cp ../rtl-sdr.rules /etc/udev/rules.d/
sudo ldconfig
cd ..
cd ..

echo "6/3 Disable built-in rtl-sdr driver"
echo 'blacklist dvb_usb_rtl28xxu' | sudo tee --append /etc/modprobe.d/blacklist-dvb_usb_rtl28xxu.conf
echo "6/4 Install SIMD FIR filter DSP library"

HOST_ARCH=$(uname -m)
if [ "$HOST_ARCH" = "x86_64" ]; then
    echo "X86 64 platform."
elif [ "$HOST_ARCH" = "armv7l" ]; then
    git clone https://github.com/projectNe10/Ne10
    cd Ne10
    mkdir build
    cd build
    export NE10_LINUX_TARGET_ARCH=armv7 
    cmake -DGNULINUX_PLATFORM=ON ..     
    make
    cp modules/libNE10.a ../../heimdall_daq_fw/Firmware/_daq_core
    cd ..
    cd ..    
else
    echo "Architecture not recognized!"
    exit
fi
echo "6/5 Install the required python3 packages"
sudo apt install python3-pip
sudo python3 -m pip install numpy
sudo python3 -m pip install configparser
sudo apt-get install libatlas-base-dev gfortran
sudo python3 -m pip install scipy
sudo python3 -m pip install pyzmq
sudo python3 -m pip install scikit-rf
# For testing
sudo python3 -m pip install plotly


sudo apt install libzmq3-dev -y
echo "6/6 Build HeIMDALL DAQ Firmware"
cd heimdall_daq_fw/Firmware/_daq_core
make

# TODO: Check installed versions:
# Scipy: 1.8 or later