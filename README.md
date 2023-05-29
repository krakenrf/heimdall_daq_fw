# HeIMDALL DAQ Firmware
Coherent data acquisition signal processing chain for multichannel SDRs.

Tested on the Raspberry Pi 4. Should work with all models, 2GB, 4GB and 8GB.
Should also be compatible with other systems including x86, but a system with at least 4-CPU cores is probably required.

## Usage

Heimdall is the data acquisition software for KrakenSDR. It needs to be used together with DSP software like our direction finding or GNU Radio software.

If you are running a Raspberry Pi 4 we recommend starting with our ready to use image at https://github.com/krakenrf/krakensdr_doa/releases/.

This image contains heimdall and the direction finding code. The direction finding software autostarts on boot if the KrakenSDR is plugged in and powered.

Alternatively, if you are on another system see our Wiki at for Docker and Virtual Box options https://github.com/krakenrf/krakensdr_docs/wiki/10.-VirtualBox,-Docker-Images-and-Install-Scripts#install-scripts, or see below for manual install information.

## Manual Installation

Manual install is only required if you are not using the premade images, and are setting up the software from a clean system. If you just want to run the DoA or PR software using a premade image please take a look at our Wiki https://github.com/krakenrf/krakensdr_docs/wiki, specifically the "Direction Finding Quickstart Guide", and the "VirtualBox, Docker Images and Install Scripts" sections.

### Install script

If a premade image does not exist for your computing device, you can use one of our install scripts to automate a fresh install. The script will install heimdall, and the DoA and PR DSP software automatically. Details on the Wiki at https://github.com/krakenrf/krakensdr_docs/wiki/10.-VirtualBox,-Docker-Images-and-Install-Scripts#install-scripts

### Manual Step by Step Install

We recommend using the install script if you are installing to a fresh system instead of doing this sytep by step install. However, if you are having problems doing the step by step install may help you figure out what is going wrong.

This code should run on any Linux system running on a aarch64(ARM64) or x86_64 systems. 

It been tested on [RaspiOS Lite 64-bit](https://downloads.raspberrypi.org/raspios_lite_arm64/images), Ubuntu 64-bit and Armbian 64-bit. 

Note that due to the use of conda, the install will only work on 64-bit systems. If you do not wish to use conda, it is possible to install to 32-bit systems. However, the reason conda is used is because the Python repo's don't appear to support numba on several ARM devices without conda. 

Steps prefixed with [ARM] should only be run on ARM systems. Steps prefixed with [x86_64] should only be run on x86_64 systems.

1. Install build dependencies
```
sudo apt update
sudo apt install build-essential git cmake libusb-1.0-0-dev lsof libzmq3-dev
```

If you are using a KerberosSDR on a Raspberry Pi 4 with the third party switches by Corey Koval, or an equivalent switch board:

```
sudo apt install pigpio
```   
   
2. Install custom KrakenRF RTL-SDR kernel driver
```    
git clone https://github.com/krakenrf/librtlsdr
cd librtlsdr
sudo cp rtl-sdr.rules /etc/udev/rules.d/rtl-sdr.rules
mkdir build
cd build
cmake ../ -DINSTALL_UDEV_RULES=ON
make
sudo ln -s ~/librtlsdr/build/src/rtl_test /usr/local/bin/kraken_test

echo 'blacklist dvb_usb_rtl28xxu' | sudo tee --append /etc/modprobe.d/blacklist-dvb_usb_rtl28xxu.conf
```

Restart the system
``` 
sudo reboot
```

3. [ARM]  Install the Ne10 DSP library for ARM devices
    
For ARM 64-bit (e.g. Running 64-Bit Raspbian OS on Pi 4) *More info on building Ne10: https://github.com/projectNe10/Ne10/blob/master/doc/building.md#building-ne10*

```
git clone https://github.com/krakenrf/Ne10
cd Ne10
mkdir build
cd build
cmake -DNE10_LINUX_TARGET_ARCH=aarch64 -DGNULINUX_PLATFORM=ON -DCMAKE_C_FLAGS="-mcpu=native -Ofast -funsafe-math-optimizations" ..
make
 ```
 
3. [X86_64] Install the KFR DSP library 
```bash
sudo apt-get install clang
```
Build and install the library
```bash
cd
git clone https://github.com/krakenrf/kfr
cd kfr
mkdir build
cd build
cmake -DENABLE_CAPI_BUILD=ON -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Release ..
make
```

Copy the built library over to the system library folder:

```
sudo cp ~/kfr/build/lib/* /usr/local/lib
```

Copy the include file over to the system includes folder:

```
sudo mkdir /usr/include/kfr
sudo cp ~/kfr/include/kfr/capi.h /usr/include/kfr
```

Run ldconfig to reset library cache:

```
sudo ldconfig
```

4. Install Miniforge

Install via the appropriate script for the system you are using (ARM aarch64 / x86_64)

[ARM]
```
cd
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
chmod ug+x Miniforge3-Linux-aarch64.sh
./Miniforge3-Linux-aarch64.sh
```
Read the license agreement and select ENTER or [yes] for all questions and wait a few minutes for the installation to complete.

[x86_64]
```
cd
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
chmod ug+x Miniforge3-Linux-x86_64.sh
./Miniforge3-Linux-x86_64.sh
```

Restart the Pi, or logout, then log on again.

```
sudo reboot
```

Disable the default base environment.

```
conda config --set auto_activate_base false
```

Restart the Pi, or logout, then log on again.

```
sudo reboot
```

5. Setup the Miniconda Environment

``` 
conda create -n kraken python=3.9.7
conda activate kraken

conda install scipy==1.9.3
conda install numba==0.56.4
conda install configparser
conda install pyzmq
conda install scikit-rf 
```

6. Create a root folder and clone the Heimdall DAQ Firmware

```
cd
mkdir krakensdr
cd krakensdr

git clone https://github.com/krakenrf/heimdall_daq_fw
cd heimdall_daq_fw
```

7. Build Heimdall C files

Browse to the _daq_core folder

```
cd ~/krakensdr/heimdall_daq_fw/Firmware/_daq_core/
```

Copy librtlsdr library and includes to the _daq_core folder

```
cp ~/librtlsdr/build/src/librtlsdr.a .
cp ~/librtlsdr/include/rtl-sdr.h .
cp ~/librtlsdr/include/rtl-sdr_export.h .
```

[ARM] If you are on an ARM device, copy the libNe10.a library over to _daq_core
```
cp ~/Ne10/build/modules/libNE10.a .
```

[PI 4 ONLY] If you are using a KerberosSDR with third party switches by Corey Koval, or equivalent, make sure you uncomment the line `PIGPIO=-lpigpio -DUSEPIGPIO` in the Makefile. If not, leave it commented out.

``` 
nano Makefile
```

Make your changes, then Ctrl+X, Y to save and exit nano.

[ALL] Now build Heimdall

``` 
make
```

## Intel Optimizations:
If you are running a machine with an Intel x86_64 CPU, you can install the highly optimized Intel MKL BLAS and Intel SVML libraries for a significant speed boost. Installing on AMD CPUs can also help.

```
conda activate kraken
conda install "blas=*=mkl"
conda install -c numba icc_rt
```

## Next Steps:

Now you will probably want to install the direction of arrival DSP code found in https://github.com/krakenrf/krakensdr_doa.

## Advanced Operation Notes:
### Test Run:
The data acquisition chain can be started by simply running the 'daq_start_sm.sh' script in sudo mode.
```bash
cd krakensdr_pr/Firmware
sudo ./daq_start_sm.sh
```

In order to start the system in simulation mode run the 'daq_synthetic_start.sh' script in sudo mode.
```bash
cd krakensdr_pr/Firmware
sudo ./daq_synthetic_start.sh
```

Prior to the system startup set parameters of the required operation mode in the 'daq_chain_config.ini'.

After starting the system the modules of the DAQ chain produce log files in the "Firmware/_logs" folder.

### Testing:
To perform unit testing on the daq chain, run the 'unit_test.sh' script. This will generate simple human readable results to the standard output. More detailed information about the results of the tests are generated into the "Firmware/_testing/test_logs" folder. It is highly recommended to use the configuration file found in the "/config_files/unit_test_k4" folder for these tests.

### Documenation:
The latest version of the documentation of the DAQ chain can be found in the Documenation folder in pdf format.


### Appendix. [IF REQUIRED] Install Python 3.8

Python 3.8 or newer is required due to its built-in shared memory library. Note that the latest Raspbian versions now come preinstalled with Python 3.8 so this step is not required.

```
sudo apt-get update
sudo apt-get install -y build-essential tk-dev libncurses5-dev libncursesw5-dev libreadline6-dev libdb5.3-dev libgdbm-dev libsqlite3-dev libssl-dev libbz2-dev libexpat1-dev liblzma-dev zlib1g-dev libffi-dev tar wget vim
wget https://www.python.org/ftp/python/3.8.0/Python-3.8.0.tgz
sudo tar zxf Python-3.8.0.tgz
cd Python-3.8.0
sudo ./configure --enable-optimizations
sudo make -j 4
sudo make install
```

Authors: Tamás Pető, Carl Laufer
