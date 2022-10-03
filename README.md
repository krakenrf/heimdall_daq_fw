# HeIMDALL DAQ Firmware
Coherent data acquisition signal processing chain for multichannel SDRs.

Tested on the Raspberry Pi 4. Should work with all models, 2GB, 4GB and 8GB.
Should also be compatible with other systems including x86, but a system with at least 4-CPU cores is probably required.

## Manual Installation

Manual install is only required if you are not using the premade images, and are setting up the software from a clean system. If you just want to run the DoA or PR software using a premade image please take a look at our Wiki https://github.com/krakenrf/krakensdr_docs/wiki, specifically the "Direction Finding Quickstart Guide", and the "VirtualBox, Docker Images and Install Scripts" sections.

### Install script

You can use on of our install scripts to automate a manual install. The script will install heimdall, and the DoA and PR DSP software. Details on the Wiki at https://github.com/krakenrf/krakensdr_docs/wiki/10.-VirtualBox,-Docker-Images-and-Install-Scripts#install-scripts

### Manual Install

This code should run on any Linux system, however it has been mostly tested on RaspiOS Lite 64-bit.

We recommend starting with a fresh install of Raspbian 64-bit Lite from https://downloads.raspberrypi.org/raspios_lite_arm64/images/

Burn the image to an 8GB or larger SD Card, connect a monitor and create a login (newer RaspiOS no longer has the default pi user). Set up WiFi with `sudo raspi-config`, enable SSH and change the hostname to "krakensdr" if desired via raspi-config.

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

3. [ARM platforms]  Install the Ne10 DSP library for ARM devices
    
*More info on building Ne10: https://github.com/projectNe10/Ne10/blob/master/doc/building.md#building-ne10*

For ARM 64-bit (e.g. Running 64-Bit Raspbian OS on Pi 4)

```
git clone https://github.com/krakenrf/Ne10
cd Ne10
mkdir build
cd build
cmake -DNE10_LINUX_TARGET_ARCH=aarch64 -DGNULINUX_PLATFORM=ON -DCMAKE_C_FLAGS="-mcpu=native -Ofast -funsafe-math-optimizations" ..
make
 ```
 
3. [X86 platforms] Install the KFR DSP library 
- Config compiler
```bash
sudo apt-get install clang
sudo update-alternatives --config c++
```
- Select clang++
*More info on the KFR library building: https://github.com/kfrlib/kfr/blob/master/README.md#usage*
- Build and install the library
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

The instructions below are for 64-bit aarch64 ARM systems such as the Pi 4. If you're installing to an x86 system, please download the appropriate miniforge installer for your system which can be found at https://github.com/conda-forge/miniforge. For x86 64-Bit systems you will most likely want https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh

```
cd
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
chmod ug+x Miniforge3-Linux-aarch64.sh
./Miniforge3-Linux-aarch64.sh
```
Read the license agreement and select ENTER or [yes] for all questions and wait a few minutes for the installation to complete.

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

conda install scipy
conda install numba
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

cp ~/librtlsdr/build/src/librtlsdr.a .
cp ~/librtlsdr/include/rtl-sdr.h .
cp ~/librtlsdr/include/rtl-sdr_export.h .

(ARM ONLY) If you are on an ARM device, copy the libNe10.a library over to _daq_core (only if you are on an ARM device and using the NE10 library)
```
cp ~/Ne10/build/modules/libNE10.a .
```

(PI 4 ONLY) If you are using a KerberosSDR with third party switches by Corey Koval, or equivalent, make sure you uncomment the line `PIGPIO=-lpigpio -DUSEPIGPIO` in the Makefile. If not, leave it commented out.

``` 
nano Makefile
```

Make your changes, then Ctrl+X, Y to save and exit nano.

(ALL) Now build Heimdall

``` 
make
```

## Intel Optimizations:
If you are running a machine with an Intel CPU, you can install the highly optimized Intel MKL BLAS and Intel SVML libraries for a significant speed boost.  

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
sudo ./Firmware/daq_start_sm.sh
```

In order to start the system in simulation mode run the 'daq_synthetic_start.sh' script in sudo mode.
```bash
sudo ./Firmware/daq_synthetic_start.sh
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
