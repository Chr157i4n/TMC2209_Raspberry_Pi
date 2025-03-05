#pylint: disable=too-many-instance-attributes
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""
Global status flags register
"""

from .bitfields import _tmc_224x_gstat as bit
from ._tmc_reg_addr import *
from .._tmc_reg import *


class GStat(TmcReg):
    """Global status flags register"""


    vm_uvlo         : bool
    register_reset  : bool
    uv_cp           : bool
    drv_err         : bool
    reset           : bool

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

        self.vm_uvlo = bool(data >> bit.vm_uvlo_bp & bit.vm_uvlo_bm)
        self.register_reset = bool(data >> bit.register_reset_bp & bit.register_reset_bm)
        self.uv_cp = bool(data >> bit.uv_cp_bp & bit.uv_cp_bm)
        self.drv_err = bool(data >> bit.drv_err_bp & bit.drv_err_bm)
        self.reset = bool(data >> bit.reset_bp & bit.reset_bm)


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        data |= self.vm_uvlo << bit.vm_uvlo_bp
        data |= self.register_reset << bit.register_reset_bp
        data |= self.uv_cp << bit.uv_cp_bp
        data |= self.drv_err << bit.drv_err_bp
        data |= self.reset << bit.reset

        return data


    def log(self, logger: TmcLogger):
        """Logs the register values

        Args:
            logger (TmcLogger): Logger
        """
        logger.log(f"Reset: {self.reset}")
        logger.log(f"Driver error: {self.drv_err}")
        logger.log(f"Under voltage on charge pump: {self.uv_cp}")
