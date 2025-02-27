#pylint: disable=too-many-instance-attributes
"""
Driver current control register
"""

from . import _tmc_2209_reg as reg
from .._tmc_logger import TMC_logger


class IHoldIRun():
    """Driver current control register"""

    data: int

    ihold: int
    irun: int
    iholddelay: int


    def __init__(self, data: int=None):
        """Initialises the object with the given register value

        Args:
            data (int, optional): register value. Defaults to None.
        """
        if data is not None:
            self.deserialise(data)


    def deserialise(self, data: int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.ihold = data >> reg.ihold_irun_ihold_bp & reg.ihold_irun_ihold_bm
        self.irun = data >> reg.ihold_irun_irun_bp & reg.ihold_irun_irun_bm
        self.iholddelay = data >> reg.ihold_irun_iholddelay_bp & reg.ihold_irun_iholddelay_bm


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        data |= self.ihold << reg.ihold_irun_ihold_bp
        data |= self.irun << reg.ihold_irun_irun_bp
        data |= self.iholddelay << reg.ihold_irun_iholddelay_bp

        return data


    def log(self, logger: TMC_logger):
        """Logs the register values

        Args:
            logger (TMC_logger): Logger
        """
        logger.log(f"IHOLD: {self.ihold}")
        logger.log(f"IRUN: {self.irun}")
        logger.log(f"IHOLDDELAY: {self.iholddelay}")
