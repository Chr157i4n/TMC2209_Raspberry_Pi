from gpiozero import LED
from bitstring import BitArray
import time
import sys
import binascii
import struct
import serial

from . import TMC_2209_reg as reg


#-----------------------------------------------------------------------
# TMC_UART
#
# this class is used to communicate with the TMC via UART
# it can be used to change the settings of the TMC.
# like the current or the microsteppingmode
#-----------------------------------------------------------------------
class TMC_UART:

    mtr_id = 0
    ser = None
    rFrame  = [0x55, 0, 0, 0  ]
    wFrame  = [0x55, 0, 0, 0 , 0, 0, 0, 0 ]
    communication_pause = 0
    

#-----------------------------------------------------------------------
# constructor
#-----------------------------------------------------------------------
    def __init__(self, serialport, baudrate, mtr_id = 0):
        try:
            self.ser = serial.Serial (serialport, baudrate)
        except Exception as e:
            errnum = e.args[0]
            print("TMC2209: SERIAL ERROR: "+str(e))
            if(errnum == 2):
                print("TMC2209: "+str(serialport)+" does not exist. You need to activate the serial port with \"sudo raspi-config\"")
            if(errnum == 13):
                print("TMC2209: you have no permission to use the serial port. You may need to add your user to the dialout group with \"sudo usermod -a -G dialout pi\"")

        self.mtr_id = mtr_id
        self.ser.BYTESIZES = 1
        self.ser.PARITIES = serial.PARITY_NONE
        self.ser.STOPBITS = 1

        self.ser.timeout = 20000/baudrate            # adjust per baud and hardware. Sequential reads without some delay fail.
        self.communication_pause = 500/baudrate     # adjust per baud and hardware. Sequential reads without some delay fail.


        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        

#-----------------------------------------------------------------------
# destructor
#-----------------------------------------------------------------------
    def __del__(self):
        if(self.ser is not None):
            self.ser.close()
        

#-----------------------------------------------------------------------
# this function calculates the crc8 parity bit
#-----------------------------------------------------------------------
    def compute_crc8_atm(self, datagram, initial_value=0):
        crc = initial_value
        # Iterate bytes in data
        for byte in datagram:
            # Iterate bits in byte
            for _ in range(0, 8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                # Shift to next bit
                byte = byte >> 1
        return crc
    

#-----------------------------------------------------------------------
# reads the registry on the TMC with a given address.
# returns the binary value of that register
#-----------------------------------------------------------------------
    def read_reg(self, register):
        
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        
        self.rFrame[1] = self.mtr_id
        self.rFrame[2] = register
        self.rFrame[3] = self.compute_crc8_atm(self.rFrame[:-1])

        rtn = self.ser.write(self.rFrame)
        if rtn != len(self.rFrame):
            print("TMC2209: Err in write {}".format(__), file=sys.stderr)
            return False

        time.sleep(self.communication_pause)  # adjust per baud and hardware. Sequential reads without some delay fail.
        
        rtn = self.ser.read(12)
        #print("received "+str(len(rtn))+" bytes; "+str(len(rtn)*8)+" bits")
        #print(rtn.hex())

        time.sleep(self.communication_pause)
        
        return(rtn[7:11])
        #return(rtn)


#-----------------------------------------------------------------------
# this function tries to read the registry of the TMC 10 times
# if a valid answer is returned, this function returns it as an integer
#-----------------------------------------------------------------------
    def read_int(self, register):
        tries = 0
        while(True):
            rtn = self.read_reg(register)
            tries += 1
            if(len(rtn)>=4):
                break
            else:
                print("TMC2209: did not get the expected 4 data bytes. Instead got "+str(len(rtn))+" Bytes")
            if(tries>=10):
                print("TMC2209: after 10 tries not valid answer. exiting")
                raise SystemExit
        
        val = struct.unpack(">i",rtn)[0]
        return(val)


#-----------------------------------------------------------------------
# this function can write a value to the register of the tmc
# 1. use read_int to get the current setting of the TMC
# 2. then modify the settings as wished
# 3. write them back to the driver with this function
#-----------------------------------------------------------------------
    def write_reg(self, register, val):
        
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        
        self.wFrame[1] = self.mtr_id
        self.wFrame[2] =  register | 0x80;  # set write bit
        
        self.wFrame[3] = 0xFF & (val>>24)
        self.wFrame[4] = 0xFF & (val>>16)
        self.wFrame[5] = 0xFF & (val>>8)
        self.wFrame[6] = 0xFF & val
        
        self.wFrame[7] = self.compute_crc8_atm(self.wFrame[:-1])


        rtn = self.ser.write(self.wFrame)
        if rtn != len(self.wFrame):
            print("TMC2209: Err in write {}".format(__), file=sys.stderr)
            return False

        time.sleep(self.communication_pause)

        return(True)


#-----------------------------------------------------------------------
# this function als writes a value to the register of the TMC
# but it also checks if the writing process was successfully by checking
# the InterfaceTransmissionCounter before and after writing
#-----------------------------------------------------------------------
    def write_reg_check(self, register, val):
        ifcnt1 = self.read_int(reg.IFCNT)
        
        tries = 0
        while(True):
            self.write_reg(register, val)
            tries += 1
            ifcnt2 = self.read_int(reg.IFCNT)
            if(ifcnt1 >= ifcnt2):
                print("TMC2209: writing not successful!")
                print("TMC2209: ifcnt:",ifcnt1,ifcnt2)
            else:
                return True
            if(tries>=10):
                print("TMC2209: after 10 tries not valid write access.")
                self.handle_error()


#-----------------------------------------------------------------------
# this function clear the communication buffers of the Raspberry Pi
#-----------------------------------------------------------------------
    def flushSerialBuffer(self):
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()


#-----------------------------------------------------------------------
# this sets a specific bit to 1
#-----------------------------------------------------------------------
    def set_bit(self, value, bit):
        return value | (bit)


#-----------------------------------------------------------------------
# this sets a specific bit to 0
#-----------------------------------------------------------------------
    def clear_bit(self, value, bit):
        return value & ~(bit)


#-----------------------------------------------------------------------
# error handling
#-----------------------------------------------------------------------
    def handle_error(self):
        gstat = self.read_int(reg.GSTAT)
        if(gstat == 0):
            print("TMC2209: Everything looks fine in GSTAT")
        if(gstat & reg.reset):
            print("TMC2209: The Driver has been reset since the last read access to GSTAT")
        if(gstat & reg.drv_err):
            print("TMC2209: The driver has been shut down due to overtemperature or short circuit detection since the last read access")
        if(gstat & reg.uv_cp):
            print("TMC2209: Undervoltage on the charge pump. The driver is disabled in this case")
        print("exiting!")
        raise SystemExit


#-----------------------------------------------------------------------
# test UART connection
#-----------------------------------------------------------------------
    def test_uart(self, register):
        
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        
        self.rFrame[1] = self.mtr_id
        self.rFrame[2] = register
        self.rFrame[3] = self.compute_crc8_atm(self.rFrame[:-1])

        rtn = self.ser.write(self.rFrame)
        if rtn != len(self.rFrame):
            print("TMC2209: Err in write {}".format(__), file=sys.stderr)
            return False

        time.sleep(self.communication_pause)  # adjust per baud and hardware. Sequential reads without some delay fail.
        
        rtn = self.ser.read(12)
        print("TMC2209: received "+str(len(rtn))+" bytes; "+str(len(rtn)*8)+" bits")
        print("TMC2209: hex: "+str(rtn.hex()))
        c = BitArray(hex=rtn.hex())
        print("TMC2209: bin: "+str(c.bin))

        time.sleep(self.communication_pause)
        
        return bytes(self.rFrame), rtn