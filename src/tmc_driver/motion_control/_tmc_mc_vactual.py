#pylint: disable=too-many-instance-attributes
#pylint: disable=too-many-arguments
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=too-many-positional-arguments
"""
VActual Motion Control module
"""

import time
from ._tmc_mc import TmcMotionControl, MovementAbsRel, StopMode
from ..com._tmc_com import TmcCom
from .._tmc_logger import Loglevel
from .. import _tmc_math as tmc_math
from ..reg._tmc_reg_addr import TmcRegAddr


class TmcMotionControlVActual(TmcMotionControl):
    """VActual Motion Control class"""

    _tmc_com:TmcCom = None

    _starttime:int = 0


    @property
    def tmc_com(self):
        """get the tmc_logger"""
        return self._tmc_com

    @tmc_com.setter
    def tmc_com(self, tmc_com):
        """set the tmc_logger"""
        self._tmc_com = tmc_com


    # def __init__(self):
    #     """constructor"""


    # def init(self):
    #     """init: called by the Tmc class"""
    #     super().init()


    def make_a_step(self):
        """method that makes on step
        """
        raise NotImplementedError


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
        rps = tmc_math.steps_to_rps(self.max_speed_fullstep, self.steps_per_rev)
        self.set_vactual_rps(rps, revolutions=steps/self.steps_per_rev)


    def set_vactual(self, vactual:int):
        """sets the register bit "VACTUAL" to to a given value
        VACTUAL allows moving the motor by UART control.
        It gives the motor velocity in +-(2^23)-1 [μsteps / t]
        0: Normal operation. Driver reacts to STEP input

        Args:
            vactual (int): value for VACTUAL
        """
        self.tmc_com.write_reg_check(TmcRegAddr.VACTUAL, vactual)


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
            self._tmc_logger.log(f"vactual: {vactual} for {duration} sec",
                                Loglevel.INFO)
        else:
            self._tmc_logger.log(f"vactual: {vactual}", Loglevel.INFO)
        self._tmc_logger.log(str(bin(vactual)), Loglevel.INFO)

        self._tmc_logger.log("writing vactual", Loglevel.INFO)
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
                # self._tmc_logger.log(f"StallGuard result: {self.get_stallguard_result()}",
                #                     Loglevel.INFO)
                time.sleep(0.1)
            if show_tstep:
                # self._tmc_logger.log(f"TStep result: {self.get_tstep()}",
                #                     Loglevel.INFO)
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
