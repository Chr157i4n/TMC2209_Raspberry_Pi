#pylint: disable=import-error
#pylint: disable=broad-exception-caught
#pylint: disable=unused-import
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=too-few-public-methods
"""
TmcComSpi stepper driver spi module
"""

import sys
import spidev
from ._tmc_com import *

# class MockSpiDev:
#     """MockSpiDev"""

#     def SpiDev(self):
#         """SpiDev"""

# try:
#     import spidev
# except ImportError:
#     print("spidev not found. Using MockSpiDev")
#     spidev = MockSpiDev()




class TmcComSpi(TmcCom):
    """TmcComSpi

    this class is used to communicate with the TMC via SPI
    it can be used to change the settings of the TMC.
    like the current or the microsteppingmode
    """

    _spi = spidev.SpiDev()
    _spi_bus: int
    _spi_dev: int

    def __init__(self,
                 spi_bus,
                 spi_dev,
                 mtr_id:int = 0,
                 tmc_logger = None
                 ):
        """constructor

        Args:
            _tmc_logger (class): TMCLogger class
            mtr_id (int, optional): driver address [0-3]. Defaults to 0.
        """
        super().__init__(mtr_id, tmc_logger)

        self._spi_bus = spi_bus
        self._spi_dev = spi_dev

        self._r_frame = [0x55, 0, 0, 0, 0]
        self._w_frame = [0x55, 0, 0, 0, 0]


    def init(self):
        """init"""
        try:
            self._spi.open(self._spi_bus, self._spi_dev)
        except Exception as e:
            self._tmc_logger.log(f"Error opening SPI: {e}", Loglevel.ERROR)
            errnum = e.args[0]
            if errnum == 2:
                self._tmc_logger.log(f"SPI Device {self._spi_dev} on Bus {self._spi_bus} does not exist.", Loglevel.ERROR)
                self._tmc_logger.log("You need to activate the SPI interface with \"sudo raspi-config\"", Loglevel.ERROR)
            sys.exit()

        self._spi.max_speed_hz =  5000
        self._spi.mode = 0b11
        self._spi.lsbfirst = False


    def __del__(self):
        """destructor"""


    def read_reg(self, addr:hex):
        """reads the registry on the TMC with a given address.
        returns the binary value of that register

        Args:
            addr (int): HEX, which register to read
        """
        self._w_frame = [addr, 0x00, 0x00, 0x00, 0x00]
        dummy_data = [0x00, 0x00, 0x00, 0x00, 0x00]

        self._spi.xfer2(self._w_frame)
        rtn = self._spi.xfer2(dummy_data)

        reset_flag =    rtn[0] >> 0 & 0x01
        driver_error =  rtn[0] >> 1 & 0x01
        sg2 =           rtn[0] >> 2 & 0x01
        standstill =    rtn[0] >> 3 & 0x01


        if reset_flag:
            self._tmc_logger.log("TMC reset flag is set", Loglevel.ERROR)
        if driver_error:
            self._tmc_logger.log("TMC driver error flag is set", Loglevel.ERROR)
        if sg2:
            self._tmc_logger.log("TMC stallguard2 flag is set", Loglevel.MOVEMENT)
        if standstill:
            self._tmc_logger.log("TMC standstill flag is set", Loglevel.MOVEMENT)


        return rtn[1:]


    def read_int(self, addr:hex, tries:int = 10):
        """this function tries to read the registry of the TMC 10 times
        if a valid answer is returned, this function returns it as an integer

        Args:
            addr (int): HEX, which register to read
            tries (int): how many tries, before error is raised (Default value = 10)
        """
        data = self.read_reg(addr)
        return int.from_bytes(data, byteorder='big', signed=False)


    def write_reg(self, addr:hex, val:int):
        """this function can write a value to the register of the tmc
        1. use read_int to get the current setting of the TMC
        2. then modify the settings as wished
        3. write them back to the driver with this function

        Args:
            addr (int): HEX, which register to write
            val (int): value for that register
        """
        self._w_frame[0] = addr | 0x80  # set write bit

        self._w_frame[1] = 0xFF & (val>>24)
        self._w_frame[2] = 0xFF & (val>>16)
        self._w_frame[3] = 0xFF & (val>>8)
        self._w_frame[4] = 0xFF & val
        # self.w_frame[7] = compute_crc8_atm(self.w_frame[:-1])

        self._spi.xfer2(self._w_frame)


    def write_reg_check(self, addr:hex, val:int, tries:int=10):
        """this function als writes a value to the register of the TMC
        but it also checks if the writing process was successfully by checking
        the InterfaceTransmissionCounter before and after writing

        Args:
            addr: HEX, which register to write
            val: value for that register
            tries: how many tries, before error is raised (Default value = 10)
        """
        self.write_reg(addr, val)


    def flush_serial_buffer(self):
        """this function clear the communication buffers of the Raspberry Pi"""


    def handle_error(self):
        """error handling"""
        raise NotImplementedError


    def test_com(self, addr):
        """test com connection

        Args:
            addr (int):  HEX, which register to test
        """
        raise NotImplementedError
