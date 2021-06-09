from gpiozero import LED
from bitstring import BitArray
import time
import sys
import binascii
import struct
import serial


class TMC_UART:

    mtr_id=0
    ser = None
    rFrame  = [0x55, 0, 0, 0  ]
    wFrame  = [0x55, 0, 0, 0 , 0, 0, 0, 0 ]
    
    def __init__(self, serialport,  baudrate):
        self.ser = serial.Serial (serialport, baudrate)
        self.mtr_id=0
        self.ser.BYTESIZES = 1
        self.ser.PARITIES = serial.PARITY_NONE
        self.ser.STOPBITS = 1
        self.ser.timeout = 0.01

        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        

    def __del__(self):
        self.ser.close()
        
    
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
    
    # -------------------------------
    def read_reg(self, reg):
        
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        
        self.rFrame[1] = self.mtr_id
        self.rFrame[2] = reg
        self.rFrame[3] = self.compute_crc8_atm(self.rFrame[:-1])

        rtn = self.ser.write(self.rFrame)
        if rtn != len(self.rFrame):
            print("TMC2209: Err in write {}".format(__), file=sys.stderr)
            return False

        time.sleep(0.001)
        
        rtn = self.ser.read(12)
        #print("received "+str(len(rtn))+" bytes; "+str(len(rtn)*8)+" bits")
        #print(rtn.hex())

        time.sleep(0.001)  # adjust per baud and hardware. Sequential reads without some delay fail.
        
        return(rtn[7:11])
        #return(rtn)

    # -------------------------------
    def read_int(self, reg):
        tries = 0
        while(True):
            rtn = self.read_reg(reg)
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

    # -------------------------------
    def write_reg(self, reg,val):
        
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        
        self.wFrame[1] = self.mtr_id
        self.wFrame[2] =  reg | 0x80;  # set write bit
        
        self.wFrame[3] = 0xFF & (val>>24)
        self.wFrame[4] = 0xFF & (val>>16)
        self.wFrame[5] = 0xFF & (val>>8)
        self.wFrame[6] = 0xFF & val
        
        self.wFrame[7] = self.compute_crc8_atm(self.wFrame[:-1])


        rtn = self.ser.write(self.wFrame)
        if rtn != len(self.wFrame):
            print("TMC2209: Err in write {}".format(__), file=sys.stderr)
            return False

        time.sleep(0.001)  # adjust per baud and hardware. 

        return(True)


    def flushSerialBuffer(self):
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()


    def set_bit(self, value, bit):
        return value | (bit)

    def clear_bit(self, value, bit):
        return value & ~(bit)



