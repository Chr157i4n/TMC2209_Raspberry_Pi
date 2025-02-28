#pylint: disable=too-many-arguments
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=too-many-instance-attributes
#pylint: disable=too-many-positional-arguments
#pylint: disable=import-outside-toplevel
#pylint: disable=bare-except
"""TMC_220X stepper driver module

this module has two different functions:
1. change setting in the TMC-driver via UART
2. move the motor via STEP/DIR pins
"""

import time
import logging
from ._tmc_gpio_board import TMC_gpio, Gpio, GpioMode, BOARD
from ._tmc_uart import TmcUart
from ._tmc_logger import TmcLogger, Loglevel
from ._tmc_move import MovementAbsRel, MovementPhase, StopMode
from . import _tmc_math as tmc_math



class Tmc220x:
    """TMC_220X

    this class has two different functions:
    1. change setting in the TMC-driver via UART
    2. move the motor via STEP/DIR pins
    """

    BOARD = BOARD
    tmc_uart = None
    tmc_logger = None
    _pin_step = -1
    _pin_dir = -1
    _pin_en = -1

    _direction = True

    _stop = StopMode.NO
    _starttime = 0


    _msres = -1
    _steps_per_rev = 0
    _fullsteps_per_rev = 0

    _current_pos = 0                 # current position of stepper in steps
    _target_pos = 0                  # the target position in steps
    _speed = 0.0                    # the current speed in steps per second
    _max_speed = 1.0                 # the maximum speed in steps per second
    _max_speed_homing = 200           # the maximum speed in steps per second for homing
    _acceleration = 1.0             # the acceleration in steps per second per second
    _acceleration_homing = 10000     # the acceleration in steps per second per second for homing
    _sqrt_twoa = 1.0                # Precomputed sqrt(2*_acceleration)
    _step_interval = 0               # the current interval between two steps
    _min_pulse_width = 1              # minimum allowed pulse with in microseconds
    _last_step_time = 0               # The last step time in microseconds
    _n = 0                          # step counter
    _c0 = 0                         # Initial step size in microseconds
    _cn = 0                         # Last step size in microseconds
    _cmin = 0                       # Min step size in microseconds based on maxSpeed
    _movement_abs_rel = MovementAbsRel.ABSOLUTE
    _movement_phase = MovementPhase.STANDSTILL

    _movement_thread = None

    _deinit_finished = False



    from ._tmc_comm import (
        read_reg, read_drv_status, read_gconf, read_gstat, clear_gstat, read_ioin, read_chopconf,
        get_direction_reg, set_direction_reg, get_iscale_analog, set_iscale_analog, get_vsense,
        set_vsense, get_internal_rsense, set_internal_rsense, set_irun_ihold, set_pdn_disable,
        set_current, get_spreadcycle, set_spreadcycle, get_interpolation, set_interpolation,
        read_microstepping_resolution, get_microstepping_resolution, set_microstepping_resolution,
        set_mstep_resolution_reg_select, get_interface_transmission_counter, get_tstep, set_vactual,
        get_stallguard_result, set_stallguard_threshold, set_coolstep_threshold,
        get_microstep_counter, get_microstep_counter_in_steps, get_toff, set_toff
    )

    from ._tmc_move import (
        set_movement_abs_rel, get_current_position, set_current_position, set_speed,
        set_speed_fullstep, set_max_speed, set_max_speed_fullstep, get_max_speed,
        set_acceleration, set_acceleration_fullstep, get_acceleration, stop, get_movement_phase,
        run_to_position_steps, run_to_position_revolutions, run_to_position_steps_threaded,
        run_to_position_revolutions_threaded, wait_for_movement_finished_threaded, run,
        distance_to_go, compute_new_speed, run_speed, make_a_step
    )

    from ._tmc_test import (
        test_pin, test_dir_step_en, test_step, test_uart
    )



    def __init__(self,
                 pin_en=-1,
                 pin_step=-1,
                 pin_dir=-1,
                 baudrate=115200,
                 serialport="/dev/serial0",
                 driver_address=0,
                 gpio_mode=None,
                 loglevel=None,
                 logprefix=None,
                 log_handlers: list = None,
                 log_formatter : logging.Formatter = None,
                 skip_uart_init: bool = False,
                 fullsteps_per_rev: int = 200
                 ):
        """constructor

        Args:
            pin_en (int): EN pin number
            pin_step (int, optional): STEP pin number. Defaults to -1.
            pin_dir (int, optional): DIR pin number. Defaults to -1.
            baudrate (int, optional): baudrate. Defaults to 115200.
            serialport (str, optional): serialport path. Defaults to "/dev/serial0".
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
            skip_uart_init (bool, optional): skip UART init. Defaults to False.
            fullsteps_per_rev (int, optional): fullsteps per revolution. Defaults to 200.
        """
        if logprefix is None:
            logprefix = f"TMC2209 {driver_address}"
        self.tmc_logger = TmcLogger(loglevel, logprefix, log_handlers, log_formatter)
        self.tmc_uart = TmcUart(self.tmc_logger, serialport, baudrate, driver_address)

        self._fullsteps_per_rev = fullsteps_per_rev

        self.tmc_logger.log("Init", Loglevel.INFO)
        TMC_gpio.init(gpio_mode)

        self.tmc_logger.log(f"EN Pin: {pin_en}", Loglevel.DEBUG)
        if pin_en != -1:
            self._pin_en = pin_en
            TMC_gpio.gpio_setup(self._pin_en, GpioMode.OUT, initial=Gpio.HIGH)

        self.tmc_logger.log(f"STEP Pin: {pin_step}", Loglevel.DEBUG)
        if pin_step != -1:
            self._pin_step = pin_step
            TMC_gpio.gpio_setup(self._pin_step, GpioMode.OUT, initial=Gpio.LOW)

        self.tmc_logger.log(f"DIR Pin: {pin_dir}", Loglevel.DEBUG)
        if pin_dir != -1:
            self._pin_dir = pin_dir
            TMC_gpio.gpio_setup(self._pin_dir, GpioMode.OUT, initial=self._direction)

        self.tmc_logger.log("GPIO Init finished", Loglevel.INFO)

        if not skip_uart_init:
            self.read_steps_per_rev()
            self.clear_gstat()

        self.tmc_uart.flush_serial_buffer()
        self.tmc_logger.log("Init finished", Loglevel.INFO)

        self.set_max_speed_fullstep(100)
        self.set_acceleration_fullstep(100)



    def __del__(self):
        """destructor"""
        if self._deinit_finished is False:
            self.tmc_logger.log("Deinit", Loglevel.INFO)

            self.set_motor_enabled(False)

            self.tmc_logger.log("GPIO cleanup", Loglevel.INFO)
            if self._pin_step != -1:
                TMC_gpio.gpio_cleanup(self._pin_step)
            if self._pin_dir != -1:
                TMC_gpio.gpio_cleanup(self._pin_dir)
            if self._pin_en != -1:
                TMC_gpio.gpio_cleanup(self._pin_en)

            self.tmc_logger.log("Deinit finished", Loglevel.INFO)
            self._deinit_finished= True
        else:
            self.tmc_logger.log("Deinit already finished", Loglevel.INFO)
        del self.tmc_uart
        del self.tmc_logger



    def set_deinitialize_true(self):
        """set deinitialize to true"""
        self._deinit_finished = True



    def set_motor_enabled(self, en):
        """enables or disables the motor current output

        Args:
            en (bool): whether the motor current output should be enabled
        """
        if self._pin_en != -1:
            TMC_gpio.gpio_output(self._pin_en, not en)
            self.tmc_logger.log(f"Motor output active: {en}", Loglevel.INFO)
        else:
            self.tmc_logger.log(f"Motor pin is: {self._pin_en}", Loglevel.INFO)



    def reverse_direction_pin(self):
        """reverses the motor shaft direction"""
        if self._pin_dir != -1:
            self._direction = not self._direction
            TMC_gpio.gpio_output(self._pin_dir, self._direction)
        else:
            self.tmc_logger.log(f"Direction pin is: {self._pin_dir}", Loglevel.INFO)



    def set_direction_pin(self, direction):
        """sets the motor shaft direction to the given value: 0 = CCW; 1 = CW

        Args:
            direction (bool): motor shaft direction: False = CCW; True = CW
        """
        if self._pin_dir != -1:
            self._direction = direction
            TMC_gpio.gpio_output(self._pin_dir, direction)
        else:
            self.tmc_logger.log(f"Direction pin is: {self._pin_dir}", Loglevel.INFO)



    def set_direction_pin_or_reg(self, direction):
        """sets the motor shaft direction to the given value: 0 = CCW; 1 = CW
        will use the reg, if pin==-1, otherwise use the pin

        Args:
            direction (bool): motor shaft direction: False = CCW; True = CW
        """
        if self._pin_dir != -1:
            self.set_direction_pin(direction)
        else:
            self.set_direction_reg(not direction) #no clue, why this has to be inverted



    def read_steps_per_rev(self):
        """returns how many steps are needed for one revolution.
        this reads the value from the tmc driver.

        Returns:
            int: Steps per revolution
        """
        self._steps_per_rev = self._fullsteps_per_rev*self.read_microstepping_resolution()
        return self._steps_per_rev



    def get_steps_per_rev(self):
        """returns how many steps are needed for one revolution.
        this gets the cached value from the library.

        Returns:
            int: Steps per revolution
        """
        return self._steps_per_rev



    def set_vactual_dur(self, vactual, duration=0, acceleration=0,
                             show_stallguard_result=False, show_tstep=False):
        """sets the register bit "VACTUAL" to to a given value
        VACTUAL allows moving the motor by UART control.
        It gives the motor velocity in +-(2^23)-1 [μsteps / t]
        0: Normal operation. Driver reacts to STEP input

        Args:
            vactual (int): value for VACTUAL
            duration (int): after this vactual will be set to 0 (Default value = 0)
            acceleration (int): use this for a velocity ramp (Default value = 0)
            show_stallguard_result (bool): prints StallGuard Result during movement
                (Default value = False)
            show_tstep (bool): prints TStep during movement (Default value = False)

        Returns:
            stop (enum): how the movement was finished
        """
        self._stop = StopMode.NO
        current_vactual = 0
        sleeptime = 0.05
        time_to_stop = 0
        if vactual<0:
            acceleration = -acceleration

        if duration != 0:
            self.tmc_logger.log(f"vactual: {vactual} for {duration} sec",
                                Loglevel.INFO)
        else:
            self.tmc_logger.log(f"vactual: {vactual}", Loglevel.INFO)
        self.tmc_logger.log(str(bin(vactual)), Loglevel.INFO)

        self.tmc_logger.log("writing vactual", Loglevel.INFO)
        if acceleration == 0:
            self.set_vactual(int(round(vactual)))

        if duration == 0:
            return -1

        self._starttime = time.time()
        current_time = time.time()
        while current_time < self._starttime+duration:
            if self._stop == StopMode.HARDSTOP:
                break
            if acceleration != 0:
                time_to_stop = self._starttime+duration-abs(current_vactual/acceleration)
                if self._stop == StopMode.SOFTSTOP:
                    time_to_stop = current_time-1
            if acceleration != 0 and current_time > time_to_stop:
                current_vactual -= acceleration*sleeptime
                self.set_vactual(int(round(current_vactual)))
                time.sleep(sleeptime)
            elif acceleration != 0 and abs(current_vactual)<abs(vactual):
                current_vactual += acceleration*sleeptime
                self.set_vactual(int(round(current_vactual)))
                time.sleep(sleeptime)
            if show_stallguard_result:
                self.tmc_logger.log(f"StallGuard result: {self.get_stallguard_result()}",
                                    Loglevel.INFO)
                time.sleep(0.1)
            if show_tstep:
                self.tmc_logger.log(f"TStep result: {self.get_tstep()}",
                                    Loglevel.INFO)
                time.sleep(0.1)
            current_time = time.time()
        self.set_vactual(0)
        return self._stop



    def set_vactual_rps(self, rps, duration=0, revolutions=0, acceleration=0):
        """converts the rps parameter to a vactual value which represents
        rotation speed in revolutions per second
        With internal oscillator:
        VACTUAL[2209] = v[Hz] / 0.715Hz

        Args:
            rps (int): value for vactual in rps
            duration (int): after this vactual will be set to 0 (Default value = 0)
            revolutions (int): after this vactual will be set to 0 (Default value = 0)
            acceleration (int): use this for a velocity ramp (Default value = 0)

        Returns:
            stop (enum): how the movement was finished
        """
        vactual = tmc_math.rps_to_vactual(rps, self._steps_per_rev)
        if revolutions!=0:
            duration = abs(revolutions/rps)
        if revolutions<0:
            vactual = -vactual
        return self.set_vactual_dur(vactual, duration, acceleration=acceleration)



    def set_vactual_rpm(self, rpm, duration=0, revolutions=0, acceleration=0):
        """converts the rps parameter to a vactual value which represents
        rotation speed in revolutions per minute

        Args:
            rpm (int): value for vactual in rpm
            duration (int): after this vactual will be set to 0 (Default value = 0)
            revolutions (int): after this vactual will be set to 0 (Default value = 0)
            acceleration (int): use this for a velocity ramp (Default value = 0)

        Returns:
            stop (enum): how the movement was finished
        """
        return self.set_vactual_rps(rpm/60, duration, revolutions, acceleration)
