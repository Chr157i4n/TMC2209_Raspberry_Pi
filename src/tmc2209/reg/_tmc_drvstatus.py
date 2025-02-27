"""
Driver Status register
"""

from . import _tmc_2209_reg as reg
from .._tmc_logger import TMC_logger, Loglevel


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

        self.stst = bool(data >> reg.stst_bp & reg.stst_bm)
        self.stealth = bool(data >> reg.stealth_bp & reg.stealth_bm)
        self.cs_actual = data >> reg.cs_actual_bp & reg.cs_actual_bm
        self.t157 = bool(data >> reg.t157_bp & reg.t157_bm)
        self.t150 = bool(data >> reg.t150_bp & reg.t150_bm)
        self.t143 = bool(data >> reg.t143_bp & reg.t143_bm)
        self.t120 = bool(data >> reg.t120_bp & reg.t120_bm)
        self.olb = bool(data >> reg.olb_bp & reg.olb_bm)
        self.ola = bool(data >> reg.ola_bp & reg.ola_bm)
        self.s2vsb = bool(data >> reg.s2vsb_bp & reg.s2vsb_bm)
        self.s2vsa = bool(data >> reg.s2vsa_bp & reg.s2vsa_bm)
        self.s2gb = bool(data >> reg.s2gb_bp & reg.s2gb_bm)
        self.s2ga = bool(data >> reg.s2ga_bp & reg.s2ga_bm)
        self.ot = bool(data >> reg.ot_bp & reg.ot_bm)
        self.otpw = bool(data >> reg.otpw_bp & reg.otpw_bm)


    def log(self, logger: TMC_logger):
        """Log the register values

        Args:
            logger (TMC_logger): logger object
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