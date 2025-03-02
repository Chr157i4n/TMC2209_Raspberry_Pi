#pylint: disable=too-many-arguments
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=too-many-instance-attributes
#pylint: disable=too-many-positional-arguments
#pylint: disable=import-outside-toplevel
#pylint: disable=bare-except
#pylint: disable=unused-import
"""TmcStepperDriver module

this module has the function to move the motor via STEP/DIR pins
"""

import logging
from ._tmc_gpio_board import Gpio, GpioMode, Board, BOARD, tmc_gpio
from .motion_control._tmc_mc import TmcMotionControl, MovementAbsRel, MovementPhase, StopMode
from .enable_control._tmc_ec import TmcEnableControl
from .enable_control._tmc_ec_pin import TmcEnableControlPin
from .motion_control._tmc_mc_step_dir import TmcMotionControlStepDir
from ._tmc_logger import TmcLogger, Loglevel
from . import _tmc_math as tmc_math



class TmcStepperDriver:
    """TmcStepperDriver

    this class has two different functions:
    1. change setting in the TMC-driver via UART
    2. move the motor via STEP/DIR pins
    """

    BOARD:Board = BOARD
    tmc_mc:TmcMotionControl = None
    tmc_ec:TmcEnableControl = None
    tmc_logger:TmcLogger = None


    _deinit_finished:bool = False



    from ._tmc_test import (
        test_step
    )


# Constructor/Destructor
# ----------------------------
    def __init__(self,
                    tmc_ec:TmcEnableControl,
                    tmc_mc:TmcMotionControl,
                    gpio_mode = None,
                    loglevel:Loglevel = Loglevel.INFO,
                    logprefix:str = None,
                    log_handlers:list = None,
                    log_formatter:logging.Formatter = None
                    ):
        """constructor

        Args:
            pin_en (int): EN pin number
            pin_step (int, optional): STEP pin number. Defaults to -1.
            pin_dir (int, optional): DIR pin number. Defaults to -1.
            tmc_com (TmcUart, optional): TMC UART object. Defaults to None.
            driver_address (int, optional): driver address [0-3]. Defaults to 0.
            gpio_mode (enum, optional): gpio mode. Defaults to None.
            loglevel (enum, optional): loglevel. Defaults to None.
            logprefix (str, optional): log prefix (name of the logger).
                Defaults to None (standard TMC prefix).
            log_handlers (list, optional): list of logging handlers.
                Defaults to None (log to console).
            log_formatter (logging.Formatter, optional): formatter for the log messages.
                Defaults to None (messages are logged in the format
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s').
        """
        if logprefix is None:
            logprefix = "StepperDriver"
        self.tmc_logger = TmcLogger(loglevel, logprefix, log_handlers, log_formatter)

        self.tmc_logger.log("Init", Loglevel.INFO)

        tmc_gpio.init(gpio_mode)

        if tmc_mc is not None:
            self.tmc_mc = tmc_mc
            self.tmc_mc.tmc_logger = self.tmc_logger
            self.tmc_mc.init()

        if tmc_ec is not None:
            self.tmc_ec = tmc_ec
            self.tmc_ec.tmc_logger = self.tmc_logger
            self.tmc_ec.init()

        self.tmc_logger.log("GPIO Init finished", Loglevel.INFO)



        self.tmc_logger.log("Init finished", Loglevel.INFO)



    def __del__(self):
        """destructor"""
        if self._deinit_finished is False:
            self.tmc_logger.log("Deinit", Loglevel.INFO)

            self.set_motor_enabled(False)

            self.tmc_logger.log("GPIO cleanup", Loglevel.INFO)

            if self._pin_en is not None:
                tmc_gpio.gpio_cleanup(self._pin_en)

            self.tmc_logger.log("Deinit finished", Loglevel.INFO)
            self._deinit_finished= True
        else:
            self.tmc_logger.log("Deinit already finished", Loglevel.INFO)
        if self.tmc_mc is not None:
            del self.tmc_mc
        if self.tmc_logger is not None:
            del self.tmc_logger



# TmcMotionControl Wrapper
# ----------------------------
    @property
    def current_pos(self):
        """_current_pos property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.current_pos
        else:
            return None

    @current_pos.setter
    def current_pos(self, current_pos:int):
        """_current_pos setter"""
        if self.tmc_mc is not None:
            self.tmc_mc.current_pos = current_pos

    @property
    def mres(self):
        """_mres property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.mres
        else:
            return None

    @mres.setter
    def mres(self, mres:int):
        """_mres setter"""
        if self.tmc_mc is not None:
            self.tmc_mc.mres = mres

    @property
    def steps_per_rev(self):
        """_steps_per_rev property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.steps_per_rev
        else:
            return None

    @property
    def fullsteps_per_rev(self):
        """_fullsteps_per_rev property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.fullsteps_per_rev
        else:
            return None

    @fullsteps_per_rev.setter
    def fullsteps_per_rev(self, fullsteps_per_rev:int):
        """_fullsteps_per_rev setter"""
        if self.tmc_mc is not None:
            self.tmc_mc.fullsteps_per_rev = fullsteps_per_rev

    @property
    def movement_abs_rel(self):
        """_movement_abs_rel property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.movement_abs_rel
        else:
            return None

    @movement_abs_rel.setter
    def movement_abs_rel(self, movement_abs_rel:MovementAbsRel):
        """_movement_abs_rel setter"""
        if self.tmc_mc is not None:
            self.tmc_mc.movement_abs_rel = movement_abs_rel

    @property
    def movement_phase(self):
        """_movement_phase property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.movement_phase
        else:
            return None

    @property
    def speed(self):
        """_speed property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.speed
        else:
            return None

    @speed.setter
    def speed(self, speed:int):
        """_speed setter"""
        if self.tmc_mc is not None:
            self.tmc_mc.speed = speed

    @property
    def max_speed(self):
        """_max_speed property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.max_speed
        else:
            return None

    @max_speed.setter
    def max_speed(self, speed:int):
        """_max_speed setter"""
        if self.tmc_mc is not None:
            self.tmc_mc.max_speed = speed

    @property
    def max_speed_fullstep(self):
        """_max_speed_fullstep property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.max_speed_fullstep
        else:
            return None

    @max_speed_fullstep.setter
    def max_speed_fullstep(self, max_speed_fullstep:int):
        """_max_speed_fullstep setter"""
        if self.tmc_mc is not None:
            self.tmc_mc.max_speed_fullstep = max_speed_fullstep

    @property
    def acceleration(self):
        """_acceleration property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.acceleration
        else:
            return None

    @acceleration.setter
    def acceleration(self, acceleration:int):
        """_acceleration setter"""
        if self.tmc_mc is not None:
            self.tmc_mc.acceleration = acceleration

    @property
    def acceleration_fullstep(self):
        """_acceleration_fullstep property"""
        if self.tmc_mc is not None:
            return self.tmc_mc.acceleration_fullstep
        else:
            return None

    @acceleration_fullstep.setter
    def acceleration_fullstep(self, acceleration_fullstep:int):
        """_acceleration_fullstep setter"""
        if self.tmc_mc is not None:
            self.tmc_mc.acceleration_fullstep = acceleration_fullstep


    def run_to_position_steps(self, steps, movement_abs_rel:MovementAbsRel = None):
        """motioncontrol wrapper"""
        if self.tmc_mc is not None:
            self.tmc_mc.run_to_position_steps(steps, movement_abs_rel)


# TmcStepperDriver methods
# ----------------------------
    def set_motor_enabled(self, en:bool):
        """enables or disables the motor current output

        Args:
            en (bool): whether the motor current output should be enabled
        """
        self.tmc_ec.set_motor_enabled(en)
