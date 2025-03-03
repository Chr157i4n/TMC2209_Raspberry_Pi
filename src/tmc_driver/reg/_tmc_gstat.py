#pylint: disable=too-many-instance-attributes
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""
Global status flags register
"""

from .bitfields import _tmc_220x_gstat as bit
from ._tmc_reg import *


class GStat(TmcReg):
    """Global status flags register"""

    data: int

    reset: bool
    drv_err: bool
    uv_cp: bool

    def __init__(self, data:int = None):
        """Initialises the object with the given register value

        Args:
            data (int): register value
        """
        self.addr = TmcRegAddr.GSTAT
        if data is not None:
            self.deserialise(data)


    def deserialise(self, data: int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.reset = bool(data >> bit.reset_bp & bit.reset_bm)
        self.drv_err = bool(data >> bit.drv_err_bp & bit.drv_err_bm)
        self.uv_cp = bool(data >> bit.uv_cp_bp & bit.uv_cp_bm)


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        data |= self.reset << bit.reset_bp
        data |= self.drv_err << bit.drv_err_bp
        data |= self.uv_cp << bit.uv_cp_bp

        return data


    def log(self, logger: TmcLogger):
        """Logs the register values

        Args:
            logger (TmcLogger): Logger
        """
        logger.log(f"Reset: {self.reset}")
        logger.log(f"Driver error: {self.drv_err}")
        logger.log(f"Under voltage on charge pump: {self.uv_cp}")
