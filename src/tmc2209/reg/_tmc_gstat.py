"""
Global status flags register
"""

from . import _tmc_2209_reg as reg
from .._tmc_logger import TMC_logger, Loglevel


class GStat():
    """Global status flags register"""

    data: int

    reset: bool
    drv_err: bool
    uv_cp: bool

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

        self.reset = bool(data >> reg.gstat_reset_bp & reg.gstat_reset_bm)
        self.drv_err = bool(data >> reg.gstat_drv_err_bp & reg.gstat_drv_err_bm)
        self.uv_cp = bool(data >> reg.gstat_uv_cp_bp & reg.gstat_uv_cp_bm)



    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        data |= self.reset << reg.gstat_reset_bp
        data |= self.drv_err << reg.gstat_drv_err_bp
        data |= self.uv_cp << reg.gstat_uv_cp_bp

        return data


    def log(self, logger: TMC_logger):
        """Logs the register values

        Args:
            logger (TMC_logger): Logger
        """
        logger.log(f"Reset: {self.reset}")
        logger.log(f"Driver error: {self.drv_err}")
        logger.log(f"Under voltage on charge pump: {self.uv_cp}")