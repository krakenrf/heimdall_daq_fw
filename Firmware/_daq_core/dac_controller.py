"""
	Description:
	Implements driver functions to controll the RF-Front modul. The driver covers the DAC controll for the IQ 
    modulators.
    TODO: Implement multiple channel operation
    TODO: Take care of bit width in case of multipple channel operation
      
    Project: HeIMDALL DAQ Firmware
    Python version: 3.6
    RF Front version: 2.0
    SBC: Asus Tinkerboard
    Author: Tamás Pető        
    License: GNU GPL V3
        
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import time
import logging
class DACController():
    def __init__(self, iface):
        """
            iface: "SPI" or "I2C"
                SPI with direct control
                I2C with using an i2c to spi bridge
        """
        self.logger = logging.getLogger(__name__)

        self.iface = iface
        self.channel_no = 6  # Used only when I2C interface is selected 
        self.bit_widths = [8, 8, 8, 8 , 8, 8]  # Should have "channel_no" items 
        self.i2c_addresses = [40, 42, 43, 44, 45, 41] # Should have "channel_no" items 
        # TODO: Load the addresses from the ini file
        
        self.init_status = False 
        if self.iface == "SPI":
            self.logger.info("Initializing SPI device")
            
            import spidev
            
            spi_speed = 5000
            self.spi_i = spidev.SpiDev()
            self.spi_q = spidev.SpiDev()
            self.spi_i.open(0, 0)
            self.spi_q.open(0, 1)
            self.spi_i.max_speed_hz = spi_speed
            self.spi_q.max_speed_hz = spi_speed
            self.spi_i.mode = 0b01
            self.spi_q.mode = 0b01
            self.spi_i.bits_per_word=8
            self.spi_q.bits_per_word=8
            self.logger.info("SPI initialization finished")
        
        elif self.iface == "I2C":
            self.logger.info("Initializing I2C device")
            
            from smbus2 import SMBus
            
            self.i2cbus = SMBus(1) # To use the i2c1 device on the ASUS Tinkerboard SBC
            
            """
                Configure spi on the i2c-spi device
                
                I DAC on SS0
                Q DAC on SS1
                According to HeIMDALL FM-RTL HW ver2.0 
                spi_config = 7 # SPI clock - 58 kHz 
                spi_config = 4 # SPI clock - 1843 kHz 
            """           
            spi_config = 4 # SPI clock - 1843 kHz 
            fid = 240 # F0h -> See SC18IS602B data sheet

            for address in self.i2c_addresses:
                self.logger.info("I2C address: {:d}".format(address))
                try:
                    self.i2cbus.write_byte_data(address , fid, spi_config)  
                    time.sleep(0.1)
                    self.init_status = True
                    self.logger.info(" I2C- SPI bridge has been configured")                
                except:
                    self.logger.critical("Error occured when writing to i2c device")
                    self.i2cbus.close()
                    self.init_status = False
                    self.logger.critical("I2C device configuration failed, device closed")
                    break
        else:
            self.logger.critical("Unidentified interface type")
            
    def close_interface(self):
        """
            Close function
            TODO:Write header
        """
        if self.iface == "SPI":
            self.spi_i.close()
            self.spi_q.close()
            self.logger.info("SPI interface closed")
        elif self.iface == "I2C":
            self.i2cbus.close()
            self.logger.info("I2C interface closed")
 
    def set_IQ_value(self, i_val, q_val, channel):
        """
            Function to set the desired IQ value using DACs
            TODO: Write header
            TODO: Implement control for multichannel op.
            TODO: Verify operation with 8 bit DACs
            
            i_val: should be in the range of 0..1
            q_val: should be in the range of 0..1
            
        Return values:
        --------------
        
            -1: Invalid I or Q value
        """
        
        # Range check
        if 1 < i_val or i_val < 0:
            self.logger.error("Ivalid I value. It sholuld be in range 0..1:", i_val)
            return -1
        if 1 < q_val or q_val < 0:
            self.logger.error("Ivalid Q value. It sholuld be in range 0..1:", q_val)
            return -1

        i_val *= 2**self.bit_widths[0]-1
        q_val *= 2**self.bit_widths[0]-1
        
        i_val = int(round(i_val))
        q_val = int(round(q_val))
        
        # Covert integers to 16 bit byte arrays
        # Covert integers to 16 bit byte arrays
        i_val <<= 14-self.bit_widths[0]           
        i_byte_array=[]
        for i in range(2):       
            b = bytes([(i_val >> 8*(1-i)) & 0xFF])
            i_byte_array.append(int.from_bytes(b,"big"))
        q_val <<= 14-self.bit_widths[0]
        q_byte_array=[]
        for q in range(2):       
            b = bytes([(q_val >> 8*(1-q)) & 0xFF])
            q_byte_array.append(int.from_bytes(b,"big"))

        # Display note: first channel is the reference channel, that is why we use (channel+1)
        self.logger.debug("Channel: {:d}, Address: {:d}".format(channel+1, self.i2c_addresses[channel]))
        self.logger.debug("Sending to I DAC: [{:d},{:d}]".format(i_byte_array[0], i_byte_array[1]))
        self.logger.debug("Sending to Q DAC: [{:d},{:d}]".format(q_byte_array[0], q_byte_array[1]))
    
        if self.iface == "SPI":
            
            self.spi_i.xfer2(i_byte_array)
            self.spi_q.xfer2(q_byte_array)
            
        elif self.iface == "I2C":            
            try:                
                self.i2cbus.write_i2c_block_data(self.i2c_addresses[channel], 1, i_byte_array)                
                self.i2cbus.write_i2c_block_data(self.i2c_addresses[channel], 2, q_byte_array)
            except:
                self.logger.error("Error occured when writing to i2c device")
                return -2
        self.logger.debug("I2C transfer completed")
        return 0


