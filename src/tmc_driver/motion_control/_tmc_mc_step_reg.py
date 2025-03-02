#pylint: disable=too-many-instance-attributes
"""
STEP/REG Motion Control module
"""

import time
import math
import threading
from ._tmc_mc import TmcMotionControl, MovementAbsRel, MovementPhase, Direction, StopMode
from ._tmc_mc_step_dir import TmcMotionControlStepDir
from .._tmc_logger import TmcLogger, Loglevel
from .._tmc_gpio_board import tmc_gpio, Gpio, GpioMode
from .. import _tmc_math as tmc_math


class TmcMotionControlStepReg(TmcMotionControlStepDir):
    """STEP/REG Motion Control class"""



    def __init__(self, pin_dir:int):
        """constructor"""
        del self._pin_step
        self._pin_dir = pin_dir


    def init(self):
        """init: called by the Tmc class"""
        self._tmc_logger.log(f"STEP Pin: {self._pin_step}", Loglevel.DEBUG)
        tmc_gpio.gpio_setup(self._pin_step, GpioMode.OUT, initial=Gpio.LOW)

        super().init()


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
        # TODO
