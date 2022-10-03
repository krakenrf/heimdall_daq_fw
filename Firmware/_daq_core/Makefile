# Makefile for compiling the DAQ firmware
# Author: Tamás Pető, Sándor Bajusz
CC=gcc
CFLAGS=-Wall -std=gnu99 -march=native -O2 -I.

# Optimized C-flags for Pi 4
#CFLAGS=-Wall -std=gnu99 -mcpu=cortex-a72 -mtune=cortex-a72 -Ofast -funsafe-math-optimizations -funroll-loops
# Uncomment if you are using third party switches from ckoval on a Rpi device and have pigpio installed
#PIGPIO=-lpigpio -DUSEPIGPIO

# Uncomment if you installed GCC 11.2
#CC=/usr/local/bin/gcc
RM= rm -f

HOST_ARCH := $(shell uname -m)

all:  daq_util rtl_daq rebuffer iq_server decimator
ifeq ($(HOST_ARCH), x86_64)
decimator: decimate_x86
else
decimator: decimate_arm_neon
endif

daq_util:
	$(CC) $(CFLAGS) -c -o ini.o ini.c
	$(CC) $(CFLAGS) -c -o log.o log.c
	$(CC) $(CFLAGS) -c -o iq_header.o iq_header.c
	$(CC) $(CFLAGS) -c -o sh_mem_util.o sh_mem_util.c

rtl_daq: iq_header.c log.c ini.c rtl_daq.c rtl_daq.h
	$(CC) $(CFLAGS) log.o ini.o iq_header.o -o rtl_daq.out rtl_daq.c -lpthread -lzmq $(PIGPIO) -L. -lrtlsdr -lusb-1.0

rebuffer: sh_mem_util.c iq_header.c log.c ini.c rebuffer.c rtl_daq.h
	$(CC) $(CFLAGS) sh_mem_util.o log.o ini.o iq_header.o -o rebuffer.out rebuffer.c -lrt -lm

decimate_x86: sh_mem_util.c iq_header.c log.c ini.c fir_decimate.c
	$(CC) $(CFLAGS) -c fir_decimate.c -o fir_decimate.o
	$(CC) $(CFLAGS) fir_decimate.o sh_mem_util.o log.o ini.o iq_header.o -o decimate.out -lrt -lkfr_capi

decimate_arm_neon: sh_mem_util.c iq_header.c log.c ini.c fir_decimate.c
	$(CC) $(CFLAGS) -DARM_NEON -c fir_decimate.c -o fir_decimate.o
	$(CC) $(CFLAGS) fir_decimate.o sh_mem_util.o log.o ini.o iq_header.o -o decimate.out -lrt -L. -lNE10 -lm

iq_server: sh_mem_util.c iq_header.c log.c ini.c iq_server.c
	$(CC) $(CFLAGS) sh_mem_util.o log.o ini.o iq_header.o -o iq_server.out iq_server.c -lrt

clean:
	$(RM) ini.o log.o iq_header.o sh_mem_util.o fir_decimate.o decimate.o rtl_daq.out rebuffer.out decimate.out iq_server.out 	

