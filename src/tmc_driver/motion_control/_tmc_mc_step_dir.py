#pylint: disable=too-many-instance-attributes
#pylint: disable=too-many-arguments
#pylint: disable=too-many-branches
#pylint: disable=too-many-positional-arguments
"""
STEP/DIR Motion Control module
"""

import time
import math
import threading
from ._tmc_mc import TmcMotionControl, MovementAbsRel, MovementPhase, Direction, StopMode
from .._tmc_logger import Loglevel
from .._tmc_gpio_board import tmc_gpio, Gpio, GpioMode
from .. import _tmc_math as tmc_math


class TmcMotionControlStepDir(TmcMotionControl):
    """STEP/DIR Motion Control class"""

    _pin_step:int = None
    _pin_dir:int = None

    _movement_thread:threading.Thread = None

    _sqrt_twoa:float = 1.0              # Precomputed sqrt(2*_acceleration)
    _step_interval:int = 0              # the current interval between two steps
    _min_pulse_width:int = 1            # minimum allowed pulse with in microseconds
    _last_step_time:int = 0             # The last step time in microseconds
    _n:int = 0                          # step counter
    _c0:int = 0                         # Initial step size in microseconds
    _cn:int = 0                         # Last step size in microseconds
    _cmin:int = 0                       # Min step size in microseconds based on maxSpeed


    @property
    def max_speed(self):
        """_max_speed property"""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, speed:int):
        """_max_speed setter"""
        speed = abs(speed)
        if self._max_speed != speed:
            self._max_speed = speed
            if speed == 0.0:
                self._cmin = 0.0
            else:
                self._cmin = 1000000.0 / speed
            # Recompute _n from current speed and adjust speed if accelerating or cruising
            if self._n > 0:
                self._n = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
                self.compute_new_speed()

    @property
    def acceleration(self):
        """_acceleration property"""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, acceleration:int):
        """_acceleration setter"""
        acceleration = abs(acceleration)
        if acceleration == self._acceleration:
            return

        # Recompute _n per Equation 17
        self._n = self._n * (self._acceleration / acceleration)
        # New c0 per Equation 7, with correction per Equation 15
        self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
        self._acceleration = acceleration
        self.compute_new_speed()

    @property
    def speed(self):
        """_speed property"""
        return self._speed

    @speed.setter
    def speed(self, speed:int):
        """_speed setter"""
        if speed == self._speed:
            return
        speed = tmc_math.constrain(speed, -self._max_speed, self._max_speed)
        if speed == 0.0:
            self._step_interval = 0
        else:
            self._step_interval = abs(1000000.0 / speed)
            if speed > 0:
                tmc_gpio.gpio_output(self._pin_dir, Gpio.HIGH)
                self._tmc_logger.log("going CW", Loglevel.MOVEMENT)
            else:
                tmc_gpio.gpio_output(self._pin_dir, Gpio.LOW)
                self._tmc_logger.log("going CCW", Loglevel.MOVEMENT)

        self._speed = speed

    @property
    def speed_fullstep(self):
        """_speed property"""
        return self._speed * self.mres

    @speed_fullstep.setter
    def speed_fullstep(self, speed:int):
        """_speed setter"""
        self.speed = speed * self.mres


    def __init__(self, pin_step:int, pin_dir:int):
        """constructor"""
        self._pin_step = pin_step
        self._pin_dir = pin_dir


    def init(self):
        """init: called by the Tmc class"""
        self._tmc_logger.log(f"STEP Pin: {self._pin_step}", Loglevel.DEBUG)
        tmc_gpio.gpio_setup(self._pin_step, GpioMode.OUT, initial=Gpio.LOW)

        self._tmc_logger.log(f"DIR Pin: {self._pin_dir}", Loglevel.DEBUG)
        tmc_gpio.gpio_setup(self._pin_dir, GpioMode.OUT, initial=self._direction.value)

        super().init()


    def __del__(self):
        """destructor"""
        if self._pin_step is not None:
            tmc_gpio.gpio_cleanup(self._pin_step)
        if self._pin_dir is not None:
            tmc_gpio.gpio_cleanup(self._pin_dir)


    def make_a_step(self):
        """method that makes on step

        for the TMC2209 there needs to be a signal duration of minimum 100 ns
        """
        tmc_gpio.gpio_output(self._pin_step, Gpio.HIGH)
        time.sleep(1/1000/1000)
        tmc_gpio.gpio_output(self._pin_step, Gpio.LOW)
        time.sleep(1/1000/1000)

        # self._tmc_logger.log("one step", Loglevel.MOVEMENT)
        self._tmc_logger.log(f"one step | cur: {self.current_pos} | tar: {self._target_pos}", Loglevel.MOVEMENT)


    def set_direction(self, direction:Direction):
        """sets the motor shaft direction to the given value: 0 = CCW; 1 = CW

        Args:
            direction (bool): motor shaft direction: False = CCW; True = CW
        """
        super().set_direction(direction)
        tmc_gpio.gpio_output(self._pin_dir, direction.value)


    def run_to_position_steps(self, steps, movement_abs_rel:MovementAbsRel = None):
        """runs the motor to the given position.
        with acceleration and deceleration
        blocks the code until finished or stopped from a different thread!
        returns true when the movement if finished normally and false,
        when the movement was stopped

        Args:
            steps (int): amount of steps; can be negative
            movement_abs_rel (enum): whether the movement should be absolut or relative
                (Default value = None)

        Returns:
            stop (enum): how the movement was finished
        """
        if movement_abs_rel is None:
            movement_abs_rel = self._movement_abs_rel

        if movement_abs_rel == MovementAbsRel.RELATIVE:
            self._target_pos = self._current_pos + steps
        else:
            self._target_pos = steps

        self._tmc_logger.log(f"cur: {self._current_pos} | tar: {self._target_pos}", Loglevel.MOVEMENT)
        # self._tmc_logger.log(f"mov: {movement_abs_rel}", Loglevel.MOVEMENT)

        self._stop = StopMode.NO
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self.compute_new_speed()
        while self.run(): #returns false, when target position is reached
            if self._stop == StopMode.HARDSTOP:
                break

        self._movement_phase = MovementPhase.STANDSTILL
        return self._stop


    def run_to_position_revolutions(self, revolutions, movement_abs_rel:MovementAbsRel = None):
        """runs the motor to the given position.
        with acceleration and deceleration
        blocks the code until finished!

        Args:
            revolutions (int): amount of revs; can be negative
            movement_abs_rel (enum): whether the movement should be absolut or relative

        Returns:
            stop (enum): how the movement was finished
        """
        return self.run_to_position_steps(round(revolutions * self._steps_per_rev),
                                            movement_abs_rel)


    def run_to_position_steps_threaded(self, steps, movement_abs_rel:MovementAbsRel = None):
        """runs the motor to the given position.
        with acceleration and deceleration
        does not block the code
        returns true when the movement if finished normally and false,
        when the movement was stopped

        Args:
            steps (int): amount of steps; can be negative
            movement_abs_rel (enum): whether the movement should be absolut or relative
                (Default value = None)

        Returns:
            stop (enum): how the movement was finished
        """
        self._movement_thread = threading.Thread(target=self.run_to_position_steps,
                                                    args=(steps, movement_abs_rel))
        self._movement_thread.start()


    def run_to_position_revolutions_threaded(self, revolutions, movement_abs_rel:MovementAbsRel = None):
        """runs the motor to the given position.
        with acceleration and deceleration
        does not block the code

        Args:
            revolutions (int): amount of revs; can be negative
            movement_abs_rel (enum): whether the movement should be absolut or relative
                (Default value = None)

        Returns:
            stop (enum): how the movement was finished
        """
        return self.run_to_position_steps_threaded(round(revolutions * self._steps_per_rev),
                                                    movement_abs_rel)


    def wait_for_movement_finished_threaded(self):
        """wait for the motor to finish the movement,
        if started threaded
        returns true when the movement if finished normally and false,
        when the movement was stopped

        Returns:
            enum: how the movement was finished
        """
        self._movement_thread.join()
        return self._stop


    def run(self):
        """calculates a new speed if a speed was made

        returns true if the target position is reached
        should not be called from outside!
        """
        if self.run_speed(): #returns true, when a step is made
            self.compute_new_speed()
        return self._speed != 0.0 and self.distance_to_go() != 0


    def distance_to_go(self):
        """returns the remaining distance the motor should run"""
        distance_to_go = self._target_pos - self._current_pos
        # self._tmc_logger.log(f"cur: {self.current_pos} | tar: {self._target_pos} | dis: {distance_to_go}", Loglevel.MOVEMENT)
        return distance_to_go


    def compute_new_speed(self):
        """returns the calculated current speed depending on the acceleration

        this code is based on:
        "Generate stepper-motor speed profiles in real time" by David Austin
        https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
        https://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
        """
        distance_to = self.distance_to_go() # +ve is clockwise from current location
        steps_to_stop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
        if (distance_to == 0 and steps_to_stop <= 2) or (self._stop == StopMode.SOFTSTOP and steps_to_stop <= 1):
            # We are at the target and its time to stop
            self._step_interval = 0
            self._speed = 0.0
            self._n = 0
            self._movement_phase = MovementPhase.STANDSTILL
            self._tmc_logger.log("time to stop", Loglevel.MOVEMENT)
            return

        if distance_to > 0:
            # We are anticlockwise from the target
            # Need to go clockwise from here, maybe decelerate now
            if self._n > 0:
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((steps_to_stop >= distance_to) or self._direction == Direction.CCW or self._stop == StopMode.SOFTSTOP):
                    self._n = -steps_to_stop # Start deceleration
                    self._movement_phase = MovementPhase.DECELERATING
            elif self._n < 0:
                # Currently decelerating, need to accel again?
                if (steps_to_stop < distance_to) and self._direction == Direction.CW:
                    self._n = -self._n # Start acceleration
                    self._movement_phase = MovementPhase.ACCELERATING
        elif distance_to < 0:
            # We are clockwise from the target
            # Need to go anticlockwise from here, maybe decelerate
            if self._n > 0:
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if (steps_to_stop >= -distance_to) or self._direction == Direction.CW or self._stop == StopMode.SOFTSTOP:
                    self._n = -steps_to_stop # Start deceleration
                    self._movement_phase = MovementPhase.DECELERATING
            elif self._n < 0:
                # Currently decelerating, need to accel again?
                if (steps_to_stop < -distance_to) and self._direction == Direction.CCW:
                    self._n = -self._n # Start acceleration
                    self._movement_phase = MovementPhase.ACCELERATING
        # Need to accelerate or decelerate
        if self._n == 0:
            # First step from stopped
            self._cn = self._c0
            tmc_gpio.gpio_output(self._pin_step, Gpio.LOW)
            if distance_to > 0:
                self.set_direction(Direction.CW)
                self._tmc_logger.log("going CW", Loglevel.MOVEMENT)
            else:
                self.set_direction(Direction.CCW)
            self._movement_phase = MovementPhase.ACCELERATING
        else:
            # Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1)) # Equation 13
            self._cn = max(self._cn, self._cmin)
            if self._cn == self._cmin:
                self._movement_phase = MovementPhase.MAXSPEED
        self._n += 1
        self._step_interval = self._cn
        self._speed = 1000000.0 / self._cn
        if self._direction == Direction.CCW:
            self._speed = -self._speed


    def run_speed(self):
        """this methods does the actual steps with the current speed"""
        # Don't do anything unless we actually have a step interval

        # self._tmc_logger.log(f"si: {self._step_interval}")

        if not self._step_interval:
            return False

        curtime = time.time_ns()/1000

        if curtime - self._last_step_time >= self._step_interval:

            self._tmc_logger.log(f"dir: {self._direction}", Loglevel.MOVEMENT)

            if self._direction == Direction.CW: # Clockwise
                self._current_pos += 1
            else: # Anticlockwise
                self._current_pos -= 1
            self.make_a_step()

            self._last_step_time = curtime # Caution: does not account for costs in step()
            return True
        return False
