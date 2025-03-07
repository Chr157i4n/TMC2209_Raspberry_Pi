#pylint: disable=import-error
#pylint: disable=broad-exception-caught
#pylint: disable=unused-import
"""
TmcCom stepper driver communication module
"""

import time
import struct
from typing import List
from .._tmc_logger import TmcLogger, Loglevel


def compute_crc8_atm(datagram, initial_value=0):
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



class TmcCom:
    """TmcCom
    """
    _tmc_logger:TmcLogger = None
    _tmc_registers = None

    mtr_id:int = 0
    r_frame:List[int]
    w_frame:List[int]
    communication_pause:int = 0
    error_handler_running:bool = False

    @property
    def tmc_logger(self):
        """get the tmc_logger"""
        return self._tmc_logger

    @tmc_logger.setter
    def tmc_logger(self, tmc_logger):
        """set the tmc_logger"""
        self._tmc_logger = tmc_logger

    @property
    def tmc_registers(self):
        """get the tmc_registers"""
        return self._tmc_registers

    @tmc_registers.setter
    def tmc_registers(self, tmc_registers):
        """set the tmc_registers"""
        self._tmc_registers = tmc_registers



    def __init__(self,
                 mtr_id:int = 0,
                 tmc_logger = None
                 ):
        """constructor

        Args:
            _tmc_logger (class): TMCLogger class
            mtr_id (int, optional): driver address [0-3]. Defaults to 0.
        """
        self._tmc_logger = tmc_logger
        self.mtr_id = mtr_id


    # def init(self):
    #     """init"""


    # def __del__(self):
    #     """destructor"""


    def read_reg(self, addr:hex):
        """reads the registry on the TMC with a given address.
        returns the binary value of that register

        Args:
            addr (int): HEX, which register to read
        """
        raise NotImplementedError


    def read_int(self, addr:hex, tries:int = 10):
        """this function tries to read the registry of the TMC 10 times
        if a valid answer is returned, this function returns it as an integer

        Args:
            addr (int): HEX, which register to read
            tries (int): how many tries, before error is raised (Default value = 10)
        """
        raise NotImplementedError


    def write_reg(self, addr:hex, val:int):
        """this function can write a value to the register of the tmc
        1. use read_int to get the current setting of the TMC
        2. then modify the settings as wished
        3. write them back to the driver with this function

        Args:
            addr (int): HEX, which register to write
            val (int): value for that register
        """
        raise NotImplementedError


    def write_reg_check(self, addr:hex, val:int, tries:int=10):
        """this function als writes a value to the register of the TMC
        but it also checks if the writing process was successfully by checking
        the InterfaceTransmissionCounter before and after writing

        Args:
            addr: HEX, which register to write
            val: value for that register
            tries: how many tries, before error is raised (Default value = 10)
        """
        raise NotImplementedError


    def flush_serial_buffer(self):
        """this function clear the communication buffers of the Raspberry Pi"""
        raise NotImplementedError


    def handle_error(self):
        """error handling"""
        raise NotImplementedError


    def test_com(self, addr):
        """test com connection

        Args:
            addr (int):  HEX, which register to test
        """
        raise NotImplementedError
