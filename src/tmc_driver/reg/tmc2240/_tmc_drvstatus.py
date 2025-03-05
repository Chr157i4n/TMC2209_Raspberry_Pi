#pylint: disable=too-many-instance-attributes
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""
Driver Status register
"""

from .bitfields import _tmc_224x_drvstatus as bit
from ._tmc_reg_addr import *
from .._tmc_reg import *


class DrvStatus(TmcReg):
    """Driver Status register"""

    stst        : bool
    olb         : bool
    ola         : bool
    s2gb        : bool
    s2ga        : bool
    otpw        : bool
    ot          : bool
    stallguard  : bool
    cs_actual   : int
    fsactive    : bool
    stealth     : bool
    s2vsb       : bool
    s2vsa       : bool
    sg_result   : int


    def __init__(self, data:int = None):
        """Initialise the DrvStatus object

        Args:
            data (int): register value
        """
        self.addr = TmcRegAddr.DRVSTATUS
        if data is not None:
            self.deserialise(data)


    def deserialise(self, data: int):
        """Deserialise the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.stst = bool(data >> bit.stst_bp & bit.stst_bm)
        self.olb = bool(data >> bit.olb_bp & bit.olb_bm)
        self.ola = bool(data >> bit.ola_bp & bit.ola_bm)
        self.s2gb = bool(data >> bit.s2gb_bp & bit.s2gb_bm)
        self.s2ga = bool(data >> bit.s2ga_bp & bit.s2ga_bm)
        self.otpw = bool(data >> bit.otpw_bp & bit.otpw_bm)
        self.ot = bool(data >> bit.ot_bp & bit.ot_bm)
        self.stallguard = bool(data >> bit.stallguard_bp & bit.stallguard_bm)
        self.cs_actual = data >> bit.cs_actual_bp & bit.cs_actual_bm
        self.fsactive = bool(data >> bit.fsactive_bp & bit.fsactive_bm)
        self.stealth = bool(data >> bit.stealth_bp & bit.stealth_bm)
        self.s2vsb = bool(data >> bit.s2vsb_bp & bit.s2vsb_bm)
        self.s2vsa = bool(data >> bit.s2vsa_bp & bit.s2vsa_bm)
        self.sg_result = data >> bit.sg_result_bp & bit.sg_result_bm


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        raise NotImplementedError


    def log(self, logger: TmcLogger):
        """Log the register values

        Args:
            logger (TmcLogger): logger object
        """
        logger.log(f"Motor is {'standing still' if self.stst else 'moving'}", Loglevel.INFO)

        logger.log(f"Motor is running on {'StealthChop' if self.stealth else 'SpreadCycle'}", Loglevel.INFO)

        logger.log(f"CS actual: {self.cs_actual}", Loglevel.INFO)

        if self.olb:
            logger.log("Open load detected on phase B", Loglevel.WARNING)

        if self.ola:
            logger.log("Open load detected on phase A", Loglevel.WARNING)

        if self.s2vsb:
            logger.log("""Short on low-side MOSFET detected on phase B.
                        The driver becomes disabled""", Loglevel.ERROR)

        if self.s2vsa:
            logger.log("""Short on low-side MOSFET detected on phase A.
                        The driver becomes disabled""", Loglevel.ERROR)

        if self.s2gb:
            logger.log("""Short to GND detected on phase B.
                                The driver becomes disabled.""", Loglevel.ERROR)

        if self.s2ga:
            logger.log("""Short to GND detected on phase A.
                                The driver becomes disabled.""", Loglevel.ERROR)

        if self.ot:
            logger.log("Driver Overheating!", Loglevel.ERROR)

        if self.otpw:
            logger.log("Driver Overheating Prewarning!", Loglevel.WARNING)
