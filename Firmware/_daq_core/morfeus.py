import hid
import time as t

class Morfeus:
    def __init__(self):
    
        self.LOmax = 5400000000  # Local Oscillator max (5400MHz)
        self.LOmin = 85000000  # Local Oscillator min (85Mhz)
        self.mil = 1000000  # Saves some zero's here and there
        self.SET = 1
        self.GET = 0

    
        self.productID = 0xeac9
        self.vendorID = 0x10c4
        self.msgArray = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.getMsg = [0, 114]
        self.setMsg = [0, 119]
        # 8 byte value carrier
        self.read_buffer = [0, 0, 0, 0, 0, 0, 0, 0]
        # 6 byte trailers
        self.sixZero = [0, 0, 0, 0, 0, 0]
        # Function Constants
        self.funcFrequency = 129
        self.funcMixGen = 130
        self.funcCurrent = 131  # 0 - 7
        self.funcBiasTee = 132  # 1 On : 0 Off
        self.funcLCD = 133  # 0 : Always on, 1 : 10s, 2 : 60s
        self.funcFW = 134
        self.funcRegister = 0
        
        self.device = self.initmorfeus()


    def findmorfeus(self):
        while True:
            try:
                count = 0
                for d in hid.enumerate(0, 0):
                    if d['product_id'] == self.productID and d['vendor_id'] == self.vendorID:
                        count += 1
                if count == 0:
                    raise OSError
                else:
                    print("Found morfeus\n")
                    return count
            except OSError:
                print('\nNo moRFeus found... Retrying in 5 seconds')
                t.sleep(5)

    def initmorfeus(self):
        devices = hid.enumerate(self.vendorID, self.productID)
        if not devices:
            raise OSError("No moRFeus device found.")
        
        mrfdevice = devices[0]
        device = hid.device()

        try:
            device.open_path(mrfdevice['path'])
            device.set_nonblocking(0)
            print("Device initialized successfully")
        except OSError as e:
            print(f"Failed to open device: {e}")
        return device
        
        
        
    def writemsgbytes(self, value, array):
        input_array = value.to_bytes(len(self.read_buffer), 'big')
        for x in range(3, 11):
            self.msgArray[x] = input_array[x - 3]
            array.append(self.msgArray[x])
        for x in range(0, 6):
            array.append(self.sixZero[x])
        self.device.write(array)    
        

    def message(self, mode, func, value):
        outputarray = []
        # this sets the mode, 0: get and 1: set
        while True:
            for x in range(0, 2):
                if mode == self.SET:
                    self.msgArray[x] = self.setMsg[x]
                    outputarray.append(self.msgArray[x])
                else:
                    self.msgArray[x] = self.getMsg[x]
                    outputarray.append(self.msgArray[x])
            # we have an variable array with our mode set...
            # now we should set the function... its always at the same position
            outputarray.append(func)
            # set the value_array
            if func == self.funcFrequency and mode == self.SET:
                freq = int(value * self.mil)
                self.writemsgbytes(freq, outputarray)
                break
            else:
                self.writemsgbytes(value, outputarray)
                break        
                

    # read function byte and return values accordingly
    def readDevice(self):
        read_array = self.device.read(16)
        if read_array:
            for x in range(3, 11):
                self.msgArray[x] = read_array[x - 1]
                # reads byte array and places it in 8 byte array to
                self.read_buffer[x - 3] = self.msgArray[x]
            init_values = int.from_bytes(self.read_buffer, byteorder='big', signed=False)
            if read_array[1] == self.funcFrequency:
                print('Freq :', str.format('{0:.6f}', init_values / self.mil))
                self.initFreq = init_values / self.mil
                return init_values / self.mil
            if read_array[1] == self.funcCurrent:
                print('Curr :', init_values)
                return init_values
            if read_array[1] == self.funcMixGen:
                if init_values == 0:
                    print("Func : Mixer")
                else:
                    print("Func : Generator")
            if read_array[1] == self.funcLCD:
                if init_values == 0:
                    print("LCD  : Always On")
                if init_values == 1:
                    print("LCD  : 10s")
                else:
                    print("LCD  : 60s")
            if read_array[1] == self.funcBiasTee:
                if init_values == 0:
                    print("Bias : Off")
                if init_values == 1:
                    print("Bias : On")         
                    
    # Get frequency from device                
    def getFreq(self):
        # Get Frequency message :
        # Object(moRFeus) sends message to device : a '0'(get) frequency
        self.message(self.GET, self.funcFrequency, 0)   
        print("Morfeus Freq: " + str(self.readDevice()))              


    # Get current from device and set Qt spinbox accordingly
    def getCur(self):
        # Get Current message :
        # Object(moRFeus) sends message to device : a '0'(get) current
        self.message(self.GET, self.funcCurrent, 0)
        # read and then set response from device
        print("Morfeus Current: " + str(self.readDevice()))
        
        
    # Set moRFeus to generator mode
    # Generator static frequency
    def genMode(self):
        #self.check5400()
        self.message(self.SET, self.funcMixGen, 1)
        #rint(self.devindex, "Generator   :")
        
    
    #def check5400(self):
    #    if self.startFreq.value() == 5400:
    #        self.moRFeus.message(self.moRFeus.SET, self.moRFeus.funcFrequency, self.moRFeus.initFreq)
    #        self.startFreq.setValue(self.moRFeus.initFreq)
    
    
    def setFreq(self, freq):
        while self.device:
            try:
                s_freq = freq
                self.message(self.SET, self.funcFrequency, s_freq)
                print("Frequency   :  {0:8.6f}".format(s_freq), "MHz")
                break
            except ValueError:
                break
        

    # Setting of the device current
    def setCurrent(self, cur=0):
        while self.device:
            if cur in range(0, 8):
                self.message(self.SET, self.funcCurrent, cur)
                print("Current     : ", cur)
                break
            else:
                print("Invalid Current Must be 0-7")
                break
