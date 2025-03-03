#pylint: disable=too-many-instance-attributes
"""
Motion Control base module
"""

from enum import Enum
from .._tmc_logger import TmcLogger, Loglevel


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



class TmcMotionControl():
    """Motion Control base class"""

    _tmc_logger:TmcLogger

    _direction:Direction = Direction.CW

    _stop:StopMode = StopMode.NO

    _current_pos:int = 0                # current position of stepper in steps
    _target_pos:int = 0                 # the target position in steps

    _speed:int = 0                      # the current speed in steps per second

    _max_speed:int = 1                  # the maximum speed in steps per second
    _max_speed_homing:int = 200         # the maximum speed in steps per second for homing
    _acceleration:int = 1               # the acceleration in steps per second per second
    _acceleration_homing:int = 10000    # the acceleration in steps per second per second for homing

    _mres:int = 2                       # microstepping resolution
    _steps_per_rev:int = 400            # microsteps per revolution
    _fullsteps_per_rev:int = 200        # fullsteps per revolution

    _movement_abs_rel:MovementAbsRel = MovementAbsRel.ABSOLUTE
    _movement_phase:MovementPhase = MovementPhase.STANDSTILL

    @property
    def tmc_logger(self):
        """_tmc_logger property"""
        return self._tmc_logger

    @tmc_logger.setter
    def tmc_logger(self, tmc_logger:TmcLogger):
        """_tmc_logger setter"""
        self._tmc_logger = tmc_logger

    @property
    def current_pos(self):
        """_current_pos property"""
        return self._current_pos

    @current_pos.setter
    def current_pos(self, current_pos:int):
        """_current_pos setter"""
        self._current_pos = current_pos

    @property
    def mres(self):
        """_mres property"""
        return self._mres

    @mres.setter
    def mres(self, mres:int):
        """_mres setter"""
        self._mres = mres
        self._steps_per_rev = self._fullsteps_per_rev * self._mres

    @property
    def steps_per_rev(self):
        """_steps_per_rev property"""
        return self._steps_per_rev

    @property
    def fullsteps_per_rev(self):
        """_fullsteps_per_rev property"""
        return self._fullsteps_per_rev

    @fullsteps_per_rev.setter
    def fullsteps_per_rev(self, fullsteps_per_rev:int):
        """_fullsteps_per_rev setter"""
        self._fullsteps_per_rev = fullsteps_per_rev
        self._steps_per_rev = self._fullsteps_per_rev * self._mres

    @property
    def movement_abs_rel(self):
        """_movement_abs_rel property"""
        return self._movement_abs_rel

    @movement_abs_rel.setter
    def movement_abs_rel(self, movement_abs_rel:MovementAbsRel):
        """_movement_abs_rel setter"""
        self._movement_abs_rel = movement_abs_rel

    @property
    def movement_phase(self):
        """_movement_phase property"""
        return self._movement_phase

    @property
    def speed(self):
        """_speed property"""
        return self._speed

    @property
    def speed_fullstep(self):
        """_speed property"""
        return self._speed * self.mres

    @property
    def max_speed(self):
        """_max_speed property"""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, speed:int):
        """_max_speed setter"""
        self._max_speed = abs(speed)

    @property
    def max_speed_fullstep(self):
        """_max_speed_fullstep property"""
        return self.max_speed * self.mres

    @max_speed_fullstep.setter
    def max_speed_fullstep(self, max_speed_fullstep:int):
        """_max_speed_fullstep setter"""
        self.max_speed = max_speed_fullstep * self.mres

    @property
    def max_speed_homing(self):
        """_max_speed_homing property"""
        return self._max_speed_homing

    @property
    def acceleration(self):
        """_acceleration property"""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, acceleration:int):
        """_acceleration setter"""
        self._acceleration = abs(acceleration)

    @property
    def acceleration_fullstep(self):
        """_acceleration_fullstep property"""
        return self._acceleration * self.mres

    @acceleration_fullstep.setter
    def acceleration_fullstep(self, acceleration_fullstep:int):
        """_acceleration_fullstep setter"""
        self.acceleration = acceleration_fullstep * self.mres


    # def __init__(self):
    #     """constructor"""


    def init(self):
        """init: called by the Tmc class"""
        self.max_speed_fullstep = 100
        self.acceleration_fullstep = 100


    def make_a_step(self):
        """make a Step"""
        raise NotImplementedError


    def set_direction(self, direction:Direction):
        """sets the motor shaft direction to the given value: 0 = CCW; 1 = CW

        Args:
            direction (bool): motor shaft direction: False = CCW; True = CW
        """
        self._direction = direction
        self._tmc_logger.log(f"New Direction is: {direction}", Loglevel.MOVEMENT)


    def stop(self, stop_mode = StopMode.HARDSTOP):
        """stop the current movement

        Args:
            stop_mode (enum): whether the movement should be stopped immediately or softly
                (Default value = StopMode.HARDSTOP)
        """
        self._stop = stop_mode


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
        raise NotImplementedError
