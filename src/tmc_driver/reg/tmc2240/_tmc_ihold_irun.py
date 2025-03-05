#pylint: disable=too-many-instance-attributes
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""
Driver current control register
"""

from .bitfields import _tmc_224x_ihold_irun as bit
from ._tmc_reg_addr import *
from .._tmc_reg import *


class IHoldIRun(TmcReg):
    """Driver current control register"""

    irundelay: int
    iholddelay: int
    irun: int
    ihold: int


    def __init__(self, data:int = None):
        """Initialises the object with the given register value

        Args:
            data (int, optional): register value. Defaults to None.
        """
        self.addr = TmcRegAddr.IHOLD_IRUN
        if data is not None:
            self.deserialise(data)


    def deserialise(self, data: int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.irundelay = data >> bit.irundelay_bp & bit.irundelay_bm
        self.iholddelay = data >> bit.iholddelay_bp & bit.iholddelay_bm
        self.irun = data >> bit.irun_bp & bit.irun_bm
        self.ihold = data >> bit.ihold_bp & bit.ihold_bm


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        data |= self.irundelay << bit.irundelay_bp
        data |= self.iholddelay << bit.iholddelay_bp
        data |= self.irun << bit.irun_bp
        data |= self.ihold << bit.ihold_bp

        return data


    def log(self, logger: TmcLogger):
        """Logs the register values

        Args:
            logger (TmcLogger): Logger
        """
        logger.log(f"IHOLD: {self.ihold}")
        logger.log(f"IRUN: {self.irun}")
        logger.log(f"IHOLDDELAY: {self.iholddelay}")
