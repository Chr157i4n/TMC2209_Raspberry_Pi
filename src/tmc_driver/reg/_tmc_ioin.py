#pylint: disable=too-many-instance-attributes
"""
INPUT register
"""

from .bitfields import _tmc_220x_ioin as bit
from ._tmc_reg import *


class IOIN(TmcReg):
    """INPUT register"""

    data: int

    enn: bool

    ms1: bool
    ms2: bool

    step: bool
    spread: bool
    dir: bool
    version: int

    def __init__(self, data:int = None):
        """Initialises the object with the given register value

        Args:
            data (int): register value
        """
        self.addr = TmcRegAddr.IOIN
        if data is not None:
            self.deserialise(data)


    def deserialise(self, data: int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.enn = bool(data >> bit.enn_bp & bit.enn_bm)
        self.ms1 = bool(data >> bit.ms1_bp & bit.ms1_bm)
        self.ms2 = bool(data >> bit.ms2_bp & bit.ms2_bm)
        self.step = bool(data >> bit.step_bp & bit.step_bm)
        self.spread = bool(data >> bit.spread_bp & bit.spread_bm)
        self.dir = bool(data >> bit.dir_bp & bit.dir_bm)
        self.version = data >> bit.version_bp & bit.version_bm


    def log(self, logger: TmcLogger):
        """Logs the register values

        Args:
            logger (TmcLogger): Logger
        """
        logger.log(f"ENN: {self.enn}")
        logger.log(f"MS1: {self.ms1}")
        logger.log(f"MS2: {self.ms2}")
        logger.log(f"STEP: {self.step}")
        logger.log(f"SPREAD: {self.spread}")
        logger.log(f"DIR: {self.dir}")
        logger.log(f"Version: {self.version}")
