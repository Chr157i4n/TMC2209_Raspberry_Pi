#pylint: disable=too-many-instance-attributes
"""
Driver Status register
"""

from .bitfields import _tmc_220x_drvstatus as bit
from .._tmc_logger import TmcLogger, Loglevel


class DrvStatus():
    """Driver Status register"""

    data: int

    stst: bool      # standstill indicator
    stealth: bool   # StealthChop indicator

    cs_actual: int  # actual motor current

    t157: bool      # 157°C comparator
    t150: bool      # 150°C comparator
    t143: bool      # 143°C comparator
    t120: bool      # 120°C comparator
    olb: bool       # open load indicator phase B
    ola: bool       # open load indicator phase A
    s2vsb: bool     # low side short indicator phase B
    s2vsa: bool     # low side short indicator phase A
    s2gb: bool      # short to ground indicator phase B
    s2ga: bool      # short to ground indicator phase A
    ot: bool        # overtemperature flag
    otpw: bool      # overtemperature prewarning flag


    def __init__(self, data: int):
        """Initialise the DrvStatus object

        Args:
            data (int): register value
        """
        self.deserialise(data)


    def deserialise(self, data: int):
        """Deserialise the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.stst = bool(data >> bit.stst_bp & bit.stst_bm)
        self.stealth = bool(data >> bit.stealth_bp & bit.stealth_bm)
        self.cs_actual = data >> bit.cs_actual_bp & bit.cs_actual_bm
        self.t157 = bool(data >> bit.t157_bp & bit.t157_bm)
        self.t150 = bool(data >> bit.t150_bp & bit.t150_bm)
        self.t143 = bool(data >> bit.t143_bp & bit.t143_bm)
        self.t120 = bool(data >> bit.t120_bp & bit.t120_bm)
        self.olb = bool(data >> bit.olb_bp & bit.olb_bm)
        self.ola = bool(data >> bit.ola_bp & bit.ola_bm)
        self.s2vsb = bool(data >> bit.s2vsb_bp & bit.s2vsb_bm)
        self.s2vsa = bool(data >> bit.s2vsa_bp & bit.s2vsa_bm)
        self.s2gb = bool(data >> bit.s2gb_bp & bit.s2gb_bm)
        self.s2ga = bool(data >> bit.s2ga_bp & bit.s2ga_bm)
        self.ot = bool(data >> bit.ot_bp & bit.ot_bm)
        self.otpw = bool(data >> bit.otpw_bp & bit.otpw_bm)


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
