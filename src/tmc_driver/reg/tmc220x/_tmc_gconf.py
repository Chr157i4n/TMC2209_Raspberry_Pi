#pylint: disable=too-many-instance-attributes
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""
General Configuration register
"""

from .bitfields import _tmc_220x_gconf as bit
from ._tmc_reg_addr import *
from .._tmc_reg import *


class GConf(TmcReg):
    """General Configuration register"""

    i_scale_analog: bool
    internal_rsense: bool
    en_spreadcycle: bool
    shaft: bool
    index_otpw: bool
    index_step: bool
    pdn_disable: bool
    mstep_reg_select: bool
    multistep_filt: bool
    test_mode: bool


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

        self.i_scale_analog = data >> bit.i_scale_analog_bp & bit.i_scale_analog_bm
        self.internal_rsense = data >> bit.internal_rsense_bp & bit.internal_rsense_bm
        self.en_spreadcycle = data >> bit.en_spreadcycle_bp & bit.en_spreadcycle_bm
        self.shaft = data >> bit.shaft_bp & bit.shaft_bm
        self.index_otpw = data >> bit.index_otpw_bp & bit.index_otpw_bm
        self.index_step = data >> bit.index_step_bp & bit.index_step_bm
        self.pdn_disable = data >> bit.pdn_disable_bp & bit.pdn_disable_bm
        self.mstep_reg_select = data >> bit.mstep_reg_select_bp & bit.mstep_reg_select_bm
        self.multistep_filt = data >> bit.multistep_filt_bp & bit.multistep_filt_bm
        self.test_mode = data >> bit.test_mode_bp & bit.test_mode_bm


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        data |= self.i_scale_analog << bit.i_scale_analog_bp
        data |= self.internal_rsense << bit.internal_rsense_bp
        data |= self.en_spreadcycle << bit.en_spreadcycle_bp
        data |= self.shaft << bit.shaft_bp
        data |= self.index_otpw << bit.index_otpw_bp
        data |= self.index_step << bit.index_step_bp
        data |= self.pdn_disable << bit.pdn_disable_bp
        data |= self.mstep_reg_select << bit.mstep_reg_select_bp
        data |= self.multistep_filt << bit.multistep_filt_bp
        data |= self.test_mode << bit.test_mode_bp

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
