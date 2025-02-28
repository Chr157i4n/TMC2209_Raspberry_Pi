#pylint: disable=import-error
#pylint: disable=broad-exception-caught
#pylint: disable=unused-import
"""
TmcSpi stepper driver spi module
"""

import time
import struct

from .reg._tmc_220x_reg_addr import TmcRegAddr
from .reg._tmc_gstat import GStat
from ._tmc_logger import Loglevel


class TmcSpi:
    """TmcSpi

    this class is used to communicate with the TMC via SPI
    it can be used to change the settings of the TMC.
    like the current or the microsteppingmode
    """
    _tmc_logger = None

    mtr_id = 0
    ser = None
    r_frame  = [0x55, 0, 0, 0  ]
    w_frame  = [0x55, 0, 0, 0 , 0, 0, 0, 0 ]
    communication_pause = 0
    error_handler_running = False

    @property
    def tmc_logger(self):
        """get the tmc_logger"""
        return self._tmc_logger

    @tmc_logger.setter
    def tmc_logger(self, tmc_logger):
        """set the tmc_logger"""
        self._tmc_logger = tmc_logger


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



    def __del__(self):
        """destructor"""
        raise NotImplementedError



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



    def read_reg(self, register:TmcRegAddr):
        """reads the registry on the TMC with a given address.
        returns the binary value of that register

        Args:
            register (int): HEX, which register to read
        """
        raise NotImplementedError



    def read_int(self, register:TmcRegAddr, tries:int = 10):
        """this function tries to read the registry of the TMC 10 times
        if a valid answer is returned, this function returns it as an integer

        Args:
            register (int): HEX, which register to read
            tries (int): how many tries, before error is raised (Default value = 10)
        """
        raise NotImplementedError



    def write_reg(self, register:TmcRegAddr, val:int):
        """this function can write a value to the register of the tmc
        1. use read_int to get the current setting of the TMC
        2. then modify the settings as wished
        3. write them back to the driver with this function

        Args:
            register (int): HEX, which register to write
            val (int): value for that register
        """
        raise NotImplementedError



    def write_reg_check(self, register:TmcRegAddr, val:int, tries:int=10):
        """this function als writes a value to the register of the TMC
        but it also checks if the writing process was successfully by checking
        the InterfaceTransmissionCounter before and after writing

        Args:
            register: HEX, which register to write
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



    def test_com(self, register:TmcRegAddr):
        """test com connection

        Args:
            register (int):  HEX, which register to read
        """
        raise NotImplementedError
