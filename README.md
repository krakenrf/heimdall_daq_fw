# HeIMDALL DAQ Firmware
Coherent data acquisition signal processing chain for multichannel SDRs

### Installation:

1. Install build dependencies
```bash
sudo apt install cmake
sudo apt install libusb-1.0-0-dev
```
   
2. Install RTL-SDR kernel driver
```bash    
git clone https://github.com/krakenrf/librtlsdr
cd rtl-sdr-kerberos
mkdir build
cd build
cmake ../ -DINSTALL_UDEV_RULES=ON
make
sudo make install
sudo cp ../rtl-sdr.rules /etc/udev/rules.d/
sudo ldconfig

echo 'blacklist dvb_usb_rtl28xxu' | sudo tee --append /etc/modprobe.d/blacklist-dvb_usb_rtl28xxu.conf
```

3. [On ARM platform only]  Install the Ne10 library for ARM devices
    
*More info on the Ne10 building: https://github.com/projectNe10/Ne10/blob/master/doc/building.md#building-ne10*

For 64-bit Pi 4 (e.g. Running 64-Bit Raspbian OS)

```bash
git clone https://github.com/krakenrf/Ne10
cd Ne10
mkdir build
cd build
cmake -DNE10_LINUX_TARGET_ARCH=aarch64 -DGNULINUX_PLATFORM=ON -DCMAKE_C_FLAGS="-mcpu=cortex-a72 -mtune=cortex-a72 -Ofast -funsafe-math-optimizations" ..
make
 ```

For 32-bit ARM systems:
```bash
git clone https://github.com/krakenrf/Ne10
cd Ne10
mkdir build
cd build
export NE10_LINUX_TARGET_ARCH=armv7 # Set the target architecture (can also be "aarch64")
cmake -DGNULINUX_PLATFORM=ON ..     # Run CMake to generate the build files
make
 ```
 
3. [On X86 platform only]. Install the KFR library 
- Config compiler
```bash
sudo apt-get install clang
sudo update-alternatives --config c++
```
- Select clang++
*More info on the KFR library building: https://github.com/kfrlib/kfr/blob/master/README.md#usage*
- Build and install the library
```bash
git clone https://github.com/kfrlib/kfr
mkdir build
cd build
cmake -DENABLE_CAPI_BUILD=ON -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
sudo ldconfig
```
- In case of cmake error, remove the problematic section from the cmake file (kfr_capi install), make the library and copy the libbrary files (libkfr_capi.so) manually to /usr/local/lib

4. If required install Python3.8 (see appendix). Most newer Raspbian builds, or modern Linux OS's will already have Python 3.8 or newer pre-installed so this step can be skipped.

5. Install Miniconda
The instructions below are so aarch64 ARM systems. If you're install to another system, please download the appropriate miniconda installer.

``` bash
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
chmod ug+x Miniforge3-Linux-aarch64.sh
./Miniforge3-Linux-aarch64.sh
```
Read the license agreement and select ENTER or [yes] for all questions.

Disable the base environment by default.

``` bash
conda config --set auto_activate_base false
```

Restart the Pi, or logout, then log on again.

``` bash
sudo reboot
```

7. Setup Miniconda Environment

``` bash
conda create -n kraken python=3.9.7
conda activate kraken

conda install scipy
conda install numba
conda install configparser
```

8. Create a root folder and clone the Heimdall DAQ Firmware

``` bash
cd
mkdir krakensdr
cd krakensdr

git clone https://github.com/krakenrf/heimdall_daq_fw
cd heimdall_daq_fw
git checkout development
```

9. Build Heimdall C files

First copy the libNe10.a library over to _daq_core

``` bash
cd heimdall_daq_fw/Firmware/_daq_core/
cp ~/Ne10/build/modules/libNE10.a .
```

Next if you are on a Pi 4, we recommend editing the Makefile to enable the optimized Pi 4 build.

``` bash
nano Makefile
```

Uncomment the line below "# Optimized C-flags for Pi 4" by deleting the '#', and comment out the top CFLAGS line by adding a '#' at the start of that line. 
Ctrl+X, Y to save and exit nano. It should look like this:

``` bash
CC=gcc
#CFLAGS=-Wall -std=gnu99 -march=native -O2
# Optimized C-flags for Pi 4
CFLAGS=-Wall -std=gnu99 -mcpu=cortex-a72 -mtune=cortex-a72 -Ofast -funsafe-math-optimizations -funroll-loops
```

Now build Heimdall

``` bash
make
```

### Next Steps:

Now you will probably want to install the direction of arrival DSP code found in https://github.com/krakenrf/krakensdr_doa.

### Advanced:
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


### Appendix. [IF REQUIRED] Install Python 3.8 [Raspberry Pi4 only]

Python 3.8 or newer is required due to its built-in shared memory library. Note that the latest Raspbian version now come preinstalled with Python 3.8 so this step is not required.

```bash
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
