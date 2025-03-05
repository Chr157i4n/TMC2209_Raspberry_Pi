#pylint: disable=too-many-instance-attributes
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""
General Configuration register
"""

from .bitfields import _tmc_224x_gconf as bit
from ._tmc_reg_addr import *
from .._tmc_reg import *


class GConf(TmcReg):
    """General Configuration register"""

    fast_standstill : bool
    en_pwm_mode     : bool
    multistep_filt  : bool
    shaft           : bool
    diag0_error     : bool
    diag0_otpw      : bool
    diag0_stall     : bool
    diag1_stall     : bool
    diag1_index     : bool
    diag1_onstate   : bool

    diag0_pushpull  : bool
    diag1_pushpull  : bool
    small_hysteresis: bool
    stop_enable     : bool
    direct_mode     : bool


    def __init__(self, data:int = None):
        """Initialises the object with the given register value

        Args:
            data (int): register value
        """
        self.addr = TmcRegAddr.GCONF
        if data is not None:
            self.deserialise(data)


    def deserialise(self, data:int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.fast_standstill = bool(data >> bit.fast_standstill_bp & bit.fast_standstill_bm)
        self.en_pwm_mode = bool(data >> bit.en_pwm_mode_bp & bit.en_pwm_mode_bm)
        self.multistep_filt = bool(data >> bit.multistep_filt_bp & bit.multistep_filt_bm)
        self.shaft = bool(data >> bit.shaft_bp & bit.shaft_bm)
        self.diag0_error = bool(data >> bit.diag0_error_bp & bit.diag0_error_bm)
        self.diag0_otpw = bool(data >> bit.diag0_otpw_bp & bit.diag0_otpw_bm)
        self.diag0_stall = bool(data >> bit.diag0_stall_bp & bit.diag0_stall_bm)
        self.diag1_stall = bool(data >> bit.diag1_stall_bp & bit.diag1_stall_bm)
        self.diag1_index = bool(data >> bit.diag1_index_bp & bit.diag1_index_bm)
        self.diag1_onstate = bool(data >> bit.diag1_onstate_bp & bit.diag1_onstate_bm)
        self.diag0_pushpull = bool(data >> bit.diag0_pushpull_bp & bit.diag0_pushpull_bm)
        self.diag1_pushpull = bool(data >> bit.diag1_pushpull_bp & bit.diag1_pushpull_bm)
        self.small_hysteresis = bool(data >> bit.small_hysteresis_bp & bit.small_hysteresis_bm)
        self.stop_enable = bool(data >> bit.stop_enable_bp & bit.stop_enable_bm)
        self.direct_mode = bool(data >> bit.direct_mode_bp & bit.direct_mode_bm)


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        data |= self.fast_standstill << bit.fast_standstill_bp
        data |= self.en_pwm_mode << bit.en_pwm_mode_bp
        data |= self.multistep_filt << bit.multistep_filt_bp
        data |= self.shaft << bit.shaft_bp
        data |= self.diag0_error << bit.diag0_error_bp
        data |= self.diag0_otpw << bit.diag0_otpw_bp
        data |= self.diag0_stall << bit.diag0_stall_bp
        data |= self.diag1_stall << bit.diag1_stall_bp
        data |= self.diag1_index << bit.diag1_index_bp
        data |= self.diag1_onstate << bit.diag1_onstate_bp
        data |= self.diag0_pushpull << bit.diag0_pushpull_bp
        data |= self.diag1_pushpull << bit.diag1_pushpull_bp
        data |= self.small_hysteresis << bit.small_hysteresis_bp
        data |= self.stop_enable << bit.stop_enable_bp
        data |= self.direct_mode << bit.direct_mode_bp

        return data


    def log(self, logger:TmcLogger):
        """Logs the register values

        Args:
            logger (TmcLogger): logger object
        """

        if self.i_scale_analog:
            logger.log("Driver is using voltage supplied to VREF as current reference",
                                Loglevel.INFO)
        else:
            logger.log("Driver is using internal reference derived from 5VOUT", Loglevel.INFO)

        if self.internal_rsense:
            logger.log("""Internal sense resistors.
                                Use current supplied into VREF as reference.""", Loglevel.WARNING)
            logger.log("VREF pin internally is driven to GND in this mode.", Loglevel.WARNING)
            logger.log("This will most likely destroy your driver!!!", Loglevel.WARNING)
            raise SystemExit
        logger.log("Operation with external sense resistors", Loglevel.INFO)

        logger.log(f"{'SpreadCycle' if self.en_spreadcycle else 'StealthChop PWM'} mode enabled", Loglevel.INFO)

        logger.log(f"{'Inverse' if self.shaft else 'Normal'} motor direction", Loglevel.INFO)

        if self.index_otpw:
            logger.log("INDEX pin outputs overtemperature prewarning flag", Loglevel.INFO)
        else:
            logger.log("INDEX shows the first microstep position of sequencer", Loglevel.INFO)

        if self.index_step:
            logger.log("INDEX output shows step pulses from internal pulse generator",
                                Loglevel.INFO)
        else:
            logger.log("INDEX output as selected by index_otpw", Loglevel.INFO)

        logger.log(f"Microstep resolution selected by {'MSTEP register' if self.mstep_reg_select else 'pins MS1, MS2'}", Loglevel.INFO)
