#pylint: disable=invalid-name
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=no-member
#pylint: disable=protected-access
"""
TMC_2209 stepper driver communication module
"""

import time
from enum import Enum
import math
import threading
from RPi import GPIO
from ._TMC_2209_logger import Loglevel



class Direction(Enum):
    """movement direction of the motor"""
    CCW = 0
    CW = 1


class MovementAbsRel(Enum):
    """movement absolute or relative"""
    ABSOLUTE = 0
    RELATIVE = 1


class MovementPhase(Enum):
    """movement phase"""
    STANDSTILL = 0
    ACCELERATING = 1
    MAXSPEED = 2
    DECELERATING = 3


class StopMode(Enum):
    """stopmode"""
    NO = 0
    SOFTSTOP = 1
    HARDSTOP = 2


def set_movement_abs_rel(self, movement_abs_rel):
    """
    set whether the movment should be relative or absolute by default.
    See the Enum MovementAbsoluteRelative

        Paramters:
            movement_abs_rel (enum): whether the movment should be relative or absolute
    """
    self._movement_abs_rel = movement_abs_rel



def get_current_position(self):
    """
    returns the current motor position in microsteps

        Returns:
            current_pos (bool): current motor position
    """
    return self._current_pos



def set_current_position(self, new_pos):
    """
    overwrites the current motor position in microsteps

        Parameters:
            new_pos (bool): new position
    """
    self._current_pos = new_pos



def set_max_speed(self, speed):
    """
    sets the maximum motor speed in µsteps per second

        Parameters:
            speed (int): maximum speed in microsteps/sec
    """
    if speed < 0.0:
        speed = -speed
    if self._max_speed != speed:
        self._max_speed = speed
        self._cmin = 1000000.0 / speed
        # Recompute _n from current speed and adjust speed if accelerating or cruising
        if self._n > 0:
            self._n = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
            self.compute_new_speed()



def set_max_speed_fullstep(self, speed):
    """
    sets the maximum motor speed in fullsteps per second

        Parameters:
            speed (int): maximum speed in fullsteps/sec
    """
    self.set_max_speed(speed*self.get_microstepping_resolution())



def get_max_speed(self):
    """
    returns the maximum motor speed in steps per second

        Returns:
            max_speed (int): current maximum speed in steps/sec
    """
    return self._max_speed



def set_acceleration(self, acceleration):
    """
    sets the motor acceleration/decceleration in µsteps per sec per sec

        Parameters:
            acceleration (int): acceleration/decceleration in µsteps per sec per sec
    """
    if acceleration == 0.0:
        return
    acceleration = abs(acceleration)
    if self._acceleration != acceleration:
        # Recompute _n per Equation 17
        self._n = self._n * (self._acceleration / acceleration)
        # New c0 per Equation 7, with correction per Equation 15
        self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
        self._acceleration = acceleration
        self.compute_new_speed()



def set_acceleration_fullstep(self, acceleration):
    """
    sets the motor acceleration/decceleration in fullsteps per sec per sec

        Parameters:
            acceleration (int): acceleration/decceleration in fullsteps per sec per sec
    """
    self.set_acceleration(acceleration*self.get_microstepping_resolution())



def get_acceleration(self):
    """
    returns the motor acceleration/decceleration in steps per sec per sec

        Returns:
            acceleration (int): acceleration/decceleration in µsteps per sec per sec
    """
    return self._acceleration



def stop(self, stop_mode = StopMode.HARDSTOP):
    """
    stop the current movement

        Parameters:
            stop_mode (enum): whether the movement should be stopped immediatly or softly
    """
    self._stop = stop_mode



def get_movement_phase(self):
    """
    return the current Movement Phase

        Returns:
            movement_phase (enum): current Movement Phase
    """
    return self._movement_phase



def run_to_position_steps(self, steps, movement_abs_rel = None):
    """
    runs the motor to the given position.
    with acceleration and deceleration
    blocks the code until finished or stopped from a different thread!
    returns true when the movement if finshed normally and false,
    when the movement was stopped

        Parameters:
            steps (int): amount of steps; can be negative
            movement_abs_rel (enum): whether the movement should be absolut or relative

        Returns:
            stop (enum): how the movement was finished
    """
    if movement_abs_rel is None:
        movement_abs_rel = self._movement_abs_rel

    if movement_abs_rel == MovementAbsRel.RELATIVE:
        self._target_pos = self._current_pos + steps
    else:
        self._target_pos = steps

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



def run_to_position_revolutions(self, revolutions, movement_absolute_relative = None):
    """
    runs the motor to the given position.
    with acceleration and deceleration
    blocks the code until finished!

        Parameters:
            revolutions (int): amount of revs; can be negative
            movement_abs_rel (enum): whether the movement should be absolut or relative

        Returns:
            stop (enum): how the movement was finished
    """
    return self.run_to_position_steps(round(revolutions * self._steps_per_rev),
                                        movement_absolute_relative)



def run_to_position_steps_threaded(self, steps, movement_abs_rel = None):
    """
    runs the motor to the given position.
    with acceleration and deceleration
    does not block the code
    returns true when the movement if finshed normally and false,
    when the movement was stopped

        Parameters:
            steps (int): amount of steps; can be negative
            movement_abs_rel (enum): whether the movement should be absolut or relative

        Returns:
            stop (enum): how the movement was finished
    """
    self._movement_thread = threading.Thread(target=self.run_to_position_steps,
                                                args=(steps, movement_abs_rel))
    self._movement_thread.start()



def run_to_position_revolutions_threaded(self, revolutions, movement_absolute_relative = None):
    """
    runs the motor to the given position.
    with acceleration and deceleration
    does not block the code

        Parameters:
            revolutions (int): amount of revs; can be negative
            movement_abs_rel (enum): whether the movement should be absolut or relative

        Returns:
            stop (enum): how the movement was finished
    """
    return self.run_to_position_steps_threaded(round(revolutions * self._steps_per_rev),
                                                movement_absolute_relative)



def wait_for_movement_finished_threaded(self):
    """
    wait for the motor to finish the movement,
    if startet threaded
    returns true when the movement if finshed normally and false,
    when the movement was stopped

        Returns:
            stop (enum): how the movement was finished
    """
    self._movement_thread.join()
    return self._stop



def run(self):
    """
    calculates a new speed if a speed was made
    returns true if the target position is reached
    should not be called from outside!
    """
    if self.run_speed(): #returns true, when a step is made
        self.compute_new_speed()
        #self.tmc_logger.log(self.get_stallguard_result())
        #self.tmc_logger.log(self.get_tstep())
    return self._speed != 0.0 and self.distance_to_go() != 0



def distance_to_go(self):
    """
    returns the remaining distance the motor should run
    """
    return self._target_pos - self._current_pos



def compute_new_speed(self):
    """
    returns the calculated current speed depending on the acceleration
    this code is based on:
    "Generate stepper-motor speed profiles in real time" by David Austin

    https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
    https://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
    """
    distance_to = self.distance_to_go() # +ve is clockwise from curent location
    steps_to_stop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
    if ((distance_to == 0 and steps_to_stop <= 2) or
    (self._stop == StopMode.SOFTSTOP and steps_to_stop <= 1)):
        # We are at the target and its time to stop
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self._movement_phase = MovementPhase.STANDSTILL
        self.tmc_logger.log("time to stop", Loglevel.MOVEMENT)
        return

    if distance_to > 0:
        # We are anticlockwise from the target
        # Need to go clockwise from here, maybe decelerate now
        if self._n > 0:
            # Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((steps_to_stop >= distance_to) or self._direction == Direction.CCW or
                self._stop == StopMode.SOFTSTOP):
                self._n = -steps_to_stop # Start deceleration
                self._movement_phase = MovementPhase.DECELERATING
        elif self._n < 0:
            # Currently decelerating, need to accel again?
            if (steps_to_stop < distance_to) and self._direction == Direction.CW:
                self._n = -self._n # Start accceleration
                self._movement_phase = MovementPhase.ACCELERATING
    elif distance_to < 0:
        # We are clockwise from the target
        # Need to go anticlockwise from here, maybe decelerate
        if self._n > 0:
            # Currently accelerating, need to decel now? Or maybe going the wrong way?
            if (((steps_to_stop >= -distance_to) or self._direction == Direction.CW or
                self._stop == StopMode.SOFTSTOP)):
                self._n = -steps_to_stop # Start deceleration
                self._movement_phase = MovementPhase.DECELERATING
        elif self._n < 0:
            # Currently decelerating, need to accel again?
            if (steps_to_stop < -distance_to) and self._direction == Direction.CCW:
                self._n = -self._n # Start accceleration
                self._movement_phase = MovementPhase.ACCELERATING
    # Need to accelerate or decelerate
    if self._n == 0:
        # First step from stopped
        self._cn = self._c0
        GPIO.output(self._pin_step, GPIO.LOW)
        #self.tmc_logger.log("distance to: " + str(distance_to))
        if distance_to > 0:
            self.set_direction_pin(1)
            self.tmc_logger.log("going CW", Loglevel.MOVEMENT)
        else:
            self.set_direction_pin(0)
            self.tmc_logger.log("going CCW", Loglevel.MOVEMENT)
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
    if self._direction == 0:
        self._speed = -self._speed



def run_speed(self):
    """
    this methods does the actual steps with the current speed
    """
    # Dont do anything unless we actually have a step interval
    if not self._step_interval:
        return False

    curtime = time.time_ns()/1000

    #self.tmc_logger.log("current time: " + str(curtime))
    #self.tmc_logger.log("last st time: " + str(self._last_step_time))

    if curtime - self._last_step_time >= self._step_interval:

        if self._direction == 1: # Clockwise
            self._current_pos += 1
        else: # Anticlockwise
            self._current_pos -= 1
        self.make_a_step()

        self._last_step_time = curtime # Caution: does not account for costs in step()
        return True
    return False



def make_a_step(self):
    """
    method that makes on step
    for the TMC2209 there needs to be a signal duration of minimum 100 ns
    """
    GPIO.output(self._pin_step, GPIO.HIGH)
    time.sleep(1/1000/1000)
    GPIO.output(self._pin_step, GPIO.LOW)
    time.sleep(1/1000/1000)

    self.tmc_logger.log("one step", Loglevel.MOVEMENT)
