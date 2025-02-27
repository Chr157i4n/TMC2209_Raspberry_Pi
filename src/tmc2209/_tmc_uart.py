#pylint: disable=import-error
#pylint: disable=broad-exception-caught
"""
TMC_UART stepper driver uart module
"""

import time
import struct
import serial

from .reg import _tmc_2209_reg as reg
from ._tmc_logger import Loglevel


class TMC_UART:
    """TMC_UART

    this class is used to communicate with the TMC via UART
    it can be used to change the settings of the TMC.
    like the current or the microsteppingmode
    """
    tmc_logger = None

    mtr_id = 0
    ser = None
    r_frame  = [0x55, 0, 0, 0  ]
    w_frame  = [0x55, 0, 0, 0 , 0, 0, 0, 0 ]
    communication_pause = 0
    error_handler_running = False



    def __init__(self,
                 tmc_logger,
                 serialport,
                 baudrate, mtr_id = 0
                 ):
        """constructor

        Args:
            tmc_logger (class): TMCLogger class
            serialport (string): serialport path
            baudrate (int): baudrate
            mtr_id (int, optional): driver address [0-3]. Defaults to 0.
        """
        self.tmc_logger = tmc_logger
        if serialport is None:
            return
        try:
            self.ser = serial.Serial(serialport, baudrate)
        except Exception as e:
            errnum = e.args[0]
            self.tmc_logger.log(f"SERIAL ERROR: {e}")
            if errnum == 2:
                self.tmc_logger.log(f""""{serialport} does not exist.
                      You need to activate the serial port with \"sudo raspi-config\"""")
            if errnum == 13:
                self.tmc_logger.log("""you have no permission to use the serial port.
                                    You may need to add your user to the dialout group
                                    with \"sudo usermod -a -G dialout pi\"""")

        self.mtr_id = mtr_id
        # adjust per baud and hardware. Sequential reads without some delay fail.
        self.communication_pause = 500 / baudrate

        if self.ser is None:
            return

        self.ser.BYTESIZES = 1
        self.ser.PARITIES = serial.PARITY_NONE
        self.ser.STOPBITS = 1

        # adjust per baud and hardware. Sequential reads without some delay fail.
        self.ser.timeout = 20000/baudrate

        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()



    def __del__(self):
        """""destructor"""""
        if self.ser is not None and type(self.ser) == serial.Serial:
            self.ser.close()



    def compute_crc8_atm(self, datagram, initial_value=0):
        """this function calculates the crc8 parity bit

        Args:
            datagram (list): datagram
            initial_value (int): initial value (Default value = 0)
        """
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



    def read_reg(self, register):
        """reads the registry on the TMC with a given address.
        returns the binary value of that register

        Args:
            register (int): HEX, which register to read
        """
        if self.ser is None:
            self.tmc_logger.log("Cannot read reg, serial is not initialized", Loglevel.ERROR)
            return False

        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()

        self.r_frame[1] = self.mtr_id
        self.r_frame[2] = register
        self.r_frame[3] = self.compute_crc8_atm(self.r_frame[:-1])

        rtn = self.ser.write(self.r_frame)
        if rtn != len(self.r_frame):
            self.tmc_logger.log("Err in write", Loglevel.ERROR)
            return False

        # adjust per baud and hardware. Sequential reads without some delay fail.
        time.sleep(self.communication_pause)

        rtn = self.ser.read(12)
        #self.tmc_logger.log(f"received {len(rtn)} bytes; {len(rtn*8)} bits")
        #self.tmc_logger.log(rtn.hex())

        time.sleep(self.communication_pause)

        return rtn



    def read_int(self, register, tries=10):
        """this function tries to read the registry of the TMC 10 times
        if a valid answer is returned, this function returns it as an integer

        Args:
            register (int): HEX, which register to read
            tries (int): how many tries, before error is raised (Default value = 10)
        """
        if self.ser is None:
            self.tmc_logger.log("Cannot read int, serial is not initialized", Loglevel.ERROR)
            return -1
        while True:
            tries -= 1
            rtn = self.read_reg(register)
            rtn_data = rtn[7:11]
            not_zero_count = len([elem for elem in rtn if elem != 0])

            if(len(rtn)<12 or not_zero_count == 0):
                self.tmc_logger.log(f"""UART Communication Error:
                                    {len(rtn_data)} data bytes |
                                    {len(rtn)} total bytes""", Loglevel.ERROR)
            elif rtn[11] != self.compute_crc8_atm(rtn[4:11]):
                self.tmc_logger.log("UART Communication Error: CRC MISMATCH", Loglevel.ERROR)
            else:
                break

            if tries<=0:
                self.tmc_logger.log("after 10 tries not valid answer", Loglevel.ERROR)
                self.tmc_logger.log(f"snd:\t{bytes(self.r_frame)}", Loglevel.DEBUG)
                self.tmc_logger.log(f"rtn:\t{rtn}", Loglevel.DEBUG)
                self.handle_error()
                return -1

        val = struct.unpack(">i",rtn_data)[0]
        return val



    def write_reg(self, register, val):
        """this function can write a value to the register of the tmc
        1. use read_int to get the current setting of the TMC
        2. then modify the settings as wished
        3. write them back to the driver with this function

        Args:
            register (int): HEX, which register to write
            val (int): value for that register
        """
        if self.ser is None:
            self.tmc_logger.log("Cannot write reg, serial is not initialized", Loglevel.ERROR)
            return False

        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()

        self.w_frame[1] = self.mtr_id
        self.w_frame[2] =  register | 0x80  # set write bit

        self.w_frame[3] = 0xFF & (val>>24)
        self.w_frame[4] = 0xFF & (val>>16)
        self.w_frame[5] = 0xFF & (val>>8)
        self.w_frame[6] = 0xFF & val

        self.w_frame[7] = self.compute_crc8_atm(self.w_frame[:-1])


        rtn = self.ser.write(self.w_frame)
        if rtn != len(self.w_frame):
            self.tmc_logger.log("Err in write", Loglevel.ERROR)
            return False

        time.sleep(self.communication_pause)

        return True



    def write_reg_check(self, register, val, tries=10):
        """this function als writes a value to the register of the TMC
        but it also checks if the writing process was successfully by checking
        the InterfaceTransmissionCounter before and after writing

        Args:
            register: HEX, which register to write
            val: value for that register
            tries: how many tries, before error is raised (Default value = 10)
        """
        if self.ser is None:
            self.tmc_logger.log("Cannot write reg check, serial is not initialized", Loglevel.ERROR)
            return False
        ifcnt1 = self.read_int(reg.IFCNT)

        if ifcnt1 == 255:
            ifcnt1 = -1

        while True:
            self.write_reg(register, val)
            tries -= 1
            ifcnt2 = self.read_int(reg.IFCNT)
            if ifcnt1 >= ifcnt2:
                self.tmc_logger.log("writing not successful!", Loglevel.ERROR)
                self.tmc_logger.log(f"ifcnt: {ifcnt1}, {ifcnt2}", Loglevel.DEBUG)
            else:
                return True
            if tries<=0:
                self.tmc_logger.log("after 10 tries no valid write access", Loglevel.ERROR)
                self.handle_error()
                return -1



    def flush_serial_buffer(self):
        """this function clear the communication buffers of the Raspberry Pi"""
        if self.ser is None:
            return
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()



    def set_bit(self, value, bit):
        """this sets a specific bit to 1

        Args:
            value: value on which the bit should be set
            bit: which bit to set
        """
        return value | (bit)



    def clear_bit(self, value, bit):
        """this sets a specific bit to 0

        Args:
            value: value on which the bit should be cleared
            bit: which bit to clear
        """
        return value & ~(bit)



    def handle_error(self):
        """error handling"""
        if self.error_handler_running:
            return
        self.error_handler_running = True
        gstat = self.read_int(reg.GSTAT)
        self.tmc_logger.log("GSTAT Error check:", Loglevel.DEBUG)
        if gstat == -1:
            self.tmc_logger.log("No answer from Driver", Loglevel.DEBUG)
        elif gstat == 0:
            self.tmc_logger.log("Everything looks fine in GSTAT", Loglevel.DEBUG)
        else:
            if gstat & reg.reset:
                self.tmc_logger.log("The Driver has been reset since the last read access to GSTAT",
                                    Loglevel.DEBUG)
            if gstat & reg.drv_err:
                self.tmc_logger.log("""The driver has been shut down due to overtemperature or short
                      circuit detection since the last read access""", Loglevel.DEBUG)
            if gstat & reg.uv_cp:
                self.tmc_logger.log("""Undervoltage on the charge pump.
                      The driver is disabled in this case""", Loglevel.DEBUG)
        self.tmc_logger.log("EXITING!", Loglevel.INFO)
        raise SystemExit



    def test_uart(self, register):
        """test UART connection

        Args:
            register (int):  HEX, which register to read
        """

        if self.ser is None:
            self.tmc_logger.log("Cannot test UART, serial is not initialized", Loglevel.ERROR)
            return False

        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()

        self.r_frame[1] = self.mtr_id
        self.r_frame[2] = register
        self.r_frame[3] = self.compute_crc8_atm(self.r_frame[:-1])

        rtn = self.ser.write(self.r_frame)
        if rtn != len(self.r_frame):
            self.tmc_logger.log("Err in write", Loglevel.ERROR)
            return False

        # adjust per baud and hardware. Sequential reads without some delay fail.
        time.sleep(self.communication_pause)

        rtn = self.ser.read(12)
        self.tmc_logger.log(f"received {len(rtn)} bytes; {len(rtn)*8} bits", Loglevel.DEBUG)
        self.tmc_logger.log(f"hex: {rtn.hex()}", Loglevel.DEBUG)
        rtn_bin = format(int(rtn.hex(),16), f"0>{len(rtn)*8}b")
        self.tmc_logger.log(f"bin: {rtn_bin}", Loglevel.DEBUG)

        time.sleep(self.communication_pause)

        return bytes(self.r_frame), rtn
