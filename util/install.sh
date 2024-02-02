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
git clone --branch v6 --single-branch 'https://github.com/kfrlib/kfr.git'
mkdir -p "kfr/build"
cd "kfr/build"
cmake -Wno-dev -GNinja -DKFR_ENABLE_CAPI_BUILD=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Release ..
ninja
sudo mkdir -p '/usr/include/kfr'
cd ..
sudo cp -v "build/lib/"* '/usr/lib'
sudo cp -v "include/kfr/capi.h" '/usr/include/kfr'
cd ..
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