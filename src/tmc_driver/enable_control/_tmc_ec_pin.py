"""
Enable Control base module
"""

from ._tmc_ec import TmcEnableControl
from .._tmc_gpio_board import Gpio, GpioMode, Board, BOARD, tmc_gpio
from .._tmc_logger import TmcLogger, Loglevel


class TmcEnableControlPin(TmcEnableControl):
    """Enable Control base class"""

    _pin_en:int = None


    def __init__(self, pin_en:int = None):
        """constructor"""
        self._pin_en = pin_en


    def init(self):
        """init: called by the Tmc class"""
        self.tmc_logger.log(f"EN Pin: {self._pin_en}", Loglevel.DEBUG)
        tmc_gpio.gpio_setup(self._pin_en, GpioMode.OUT, initial=Gpio.HIGH)


    def __del__(self):
        """destructor"""
        if self._pin_en is not None:
            tmc_gpio.gpio_cleanup(self._pin_en)


    def set_motor_enabled(self, en):
        """enables or disables the motor current output

        Args:
            en (bool): whether the motor current output should be enabled
        """
        if self._pin_en is not None:
            tmc_gpio.gpio_output(self._pin_en, not en)
            self.tmc_logger.log(f"Motor output active: {en}", Loglevel.INFO)
