# HeIMDALL DAQ Firmware
Coherent data acquisition signal processing chain for multichannel SDRs

### Installation:

1. Install build dependencies
```bash
	apt install cmake
    apt install libusb-1.0-0-dev
```
   
2. Install RTl-SDR kernel driver
```bash    
    git clone https://github.com/rtlsdrblog/rtl-sdr-kerberos
    cd rtl-sdr-kerberos
    mkdir build
    cd build
    cmake ../ -DINSTALL_UDEV_RULES=ON
    make
    sudo make install
    sudo cp ../rtl-sdr.rules /etc/udev/rules.d/
    sudo ldconfig
```
3. Disable builtin rtl-sdr drivers
```bash   
    echo 'blacklist dvb_usb_rtl28xxu' | sudo tee --append /etc/modprobe.d/blacklist-dvb_usb_rtl28xxu.conf
```

4. [On ARM platform only]  Install the Ne10 library for ARM devices
    
    *More info on the Ne10 building: https://github.com/projectNe10/Ne10/blob/master/doc/building.md#building-ne10*
```bash
    git clone https://github.com/krakenrf/Ne10
    cd Ne10
    mkdir build
    cd build
    export NE10_LINUX_TARGET_ARCH=armv7 # Set the target architecture (can also be "aarch64")
    cmake -DGNULINUX_PLATFORM=ON ..     # Run CMake to generate the build files
    make
 ```
 *copy "NE10_PATH/build/modules/libNE10.a" to "Firmware/_daq_core/'*
 
 For 64-bit Pi 4:
 ```bash
    git clone https://github.com/krakenrf/Ne10
    cd Ne10
    mkdir build
    cd build
    cmake -DNE10_LINUX_TARGET_ARCH=aarch64 -DGNULINUX_PLATFORM=ON -DCMAKE_C_FLAGS="-mcpu=cortex-a72 -mtune=cortex-a72 -Ofast -funsafe-math-optimizations" ..
    make
 ```
 
 
4. [On X86 platform only]. Install the KFR library 
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

5. Install Python 3.8 [Raspberry Pi4 only]

	Python 3.8 or newer is required due to its built-in shared memory library
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
6. Install the required python packages
    ```bash
    sudo python3 -m pip install numpy
    sudo python3 -m pip install configparser
    # For testing
    sudo apt-get install libatlas-base-dev gfortran
    sudo python3 -m pip install scipy
    sudo python3 -m pip install plotly
    ```
 7. Build the modules of the daq firmware
 	``` bash
    cd Firmware/_daq_core
    make
    ```
### Run:
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

Authors: Tamás Pető, Carl Laufer
