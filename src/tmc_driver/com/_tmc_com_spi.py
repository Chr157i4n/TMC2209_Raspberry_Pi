#pylint: disable=import-error
#pylint: disable=broad-exception-caught
#pylint: disable=unused-import
#pylint: disable=duplicate-code
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""
TmcComSpi stepper driver spi module
"""


from ._tmc_com import *


class TmcComSpi(TmcCom):
    """TmcComSpi

    this class is used to communicate with the TMC via SPI
    it can be used to change the settings of the TMC.
    like the current or the microsteppingmode
    """


    def __init__(self,
                 mtr_id:int = 0,
                 tmc_logger = None
                 ):
        """constructor

        Args:
            _tmc_logger (class): TMCLogger class
            mtr_id (int, optional): driver address [0-3]. Defaults to 0.
        """
        super().__init__(mtr_id, tmc_logger)
        raise NotImplementedError



    def __del__(self):
        """destructor"""


    def read_reg(self, register):
        """reads the registry on the TMC with a given address.
        returns the binary value of that register

        Args:
            register (int): HEX, which register to read
        """
        raise NotImplementedError


    def read_int(self, register, tries:int = 10):
        """this function tries to read the registry of the TMC 10 times
        if a valid answer is returned, this function returns it as an integer

        Args:
            register (int): HEX, which register to read
            tries (int): how many tries, before error is raised (Default value = 10)
        """
        raise NotImplementedError


    def write_reg(self, register, val:int):
        """this function can write a value to the register of the tmc
        1. use read_int to get the current setting of the TMC
        2. then modify the settings as wished
        3. write them back to the driver with this function

        Args:
            register (int): HEX, which register to write
            val (int): value for that register
        """
        raise NotImplementedError


    def write_reg_check(self, register, val:int, tries:int=10):
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


    def test_com(self, register):
        """test com connection

        Args:
            register (int):  HEX, which register to read
        """
        raise NotImplementedError
