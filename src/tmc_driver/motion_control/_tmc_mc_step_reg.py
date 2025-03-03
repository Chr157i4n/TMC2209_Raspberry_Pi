#pylint: disable=too-many-instance-attributes
"""
STEP/REG Motion Control module
"""

from ._tmc_mc import Direction
from ._tmc_mc_step_dir import TmcMotionControlStepDir
from ..com._tmc_com import TmcCom
from .._tmc_logger import Loglevel
from .._tmc_gpio_board import tmc_gpio, Gpio, GpioMode
from ..reg._tmc_gconf import GConf


class TmcMotionControlStepReg(TmcMotionControlStepDir):
    """STEP/REG Motion Control class"""

    _tmc_com:TmcCom = None


    @property
    def tmc_com(self):
        """get the tmc_logger"""
        return self._tmc_com

    @tmc_com.setter
    def tmc_com(self, tmc_com):
        """set the tmc_logger"""
        self._tmc_com = tmc_com


    def __init__(self, pin_step:int):
        """constructor"""
        super().__init__(pin_step, None)
        del self._pin_dir


    def init(self):
        """init: called by the Tmc class"""
        self._tmc_logger.log(f"STEP Pin: {self._pin_step}", Loglevel.DEBUG)
        tmc_gpio.gpio_setup(self._pin_step, GpioMode.OUT, initial=Gpio.LOW)

        self.max_speed_fullstep = 100
        self.acceleration_fullstep = 100


    def __del__(self):
        """destructor"""
        if self._pin_step is not None:
            tmc_gpio.gpio_cleanup(self._pin_step)


    def set_direction(self, direction:Direction):
        """sets the motor shaft direction to the given value: 0 = CCW; 1 = CW

        Args:
            direction (bool): motor shaft direction: False = CCW; True = CW
        """
        self._direction = direction
        self._tmc_logger.log(f"New Direction is: {direction}", Loglevel.MOVEMENT)
        gconf = GConf()
        gconf.read(self._tmc_com)

        gconf.shaft = bool(direction.value)

        gconf.write(self._tmc_com)
