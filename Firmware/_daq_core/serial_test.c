
#include <stdio.h>
#include <unistd.h>

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#include "log.h"
#include "rtl-sdr.h"
#include "rtl_daq.h"

int main( int argc, char** argv )
{   
    log_set_level(LOG_TRACE);
        
    
    log_info("Starting serial test");
    // Get device index by serial number
    char dev_serial[16];
    int i = 0;
    sprintf(dev_serial, "%d", 1000+i);
    int dev_index = rtlsdr_get_index_by_serial(dev_serial);
    log_info("Device serial:%s, index: %d",dev_serial, dev_index);    
    if(dev_index==-3){log_fatal("Device with the requested serial number is not available");}

    // Open device and get serial number
    rtlsdr_dev_t *dev = NULL;
    if (rtlsdr_open(&dev, 0) !=0)
    {
        log_fatal("Failed to open RTL-SDR device: %s", strerror(errno));
        return -1;
    }
    
    char manufact[128];
    char product[128];
    char serial[128];
    rtlsdr_get_usb_strings(dev,manufact,product,serial);
    log_info("Serial number of the currently active device is: %s", serial);

    rtlsdr_close(dev);
    log_info("All the resources are free now");
    
    return 0;
}

