#pylint: disable=too-many-instance-attributes
"""
INPUT register
"""

from . import _tmc_2209_reg as reg
from .._tmc_logger import TMC_logger


class IOIN():
    """INPUT register"""

    data: int

    enn: bool

    ms1: bool
    ms2: bool

    step: bool
    spread: bool
    dir: bool
    version: int

    def __init__(self, data: int):
        """Initialises the object with the given register value

        Args:
            data (int): register value
        """
        self.deserialise(data)


    def deserialise(self, data: int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.enn = bool(data >> reg.io_enn_bp & reg.io_enn_bm)
        self.ms1 = bool(data >> reg.io_ms1_bp & reg.io_ms1_bm)
        self.ms2 = bool(data >> reg.io_ms2_bp & reg.io_ms2_bm)
        self.step = bool(data >> reg.io_step_bp & reg.io_step_bm)
        self.spread = bool(data >> reg.io_spread_bp & reg.io_spread_bm)
        self.dir = bool(data >> reg.io_dir_bp & reg.io_dir_bm)
        self.version = data >> reg.io_version_bp & reg.io_version_bm


    def log(self, logger: TMC_logger):
        """Logs the register values

        Args:
            logger (TMC_logger): Logger
        """
        logger.log(f"ENN: {self.enn}")
        logger.log(f"MS1: {self.ms1}")
        logger.log(f"MS2: {self.ms2}")
        logger.log(f"STEP: {self.step}")
        logger.log(f"SPREAD: {self.spread}")
        logger.log(f"DIR: {self.dir}")
        logger.log(f"Version: {self.version}")
