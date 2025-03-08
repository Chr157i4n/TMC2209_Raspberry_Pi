"""
Enable Control base module
"""

from ._tmc_ec import TmcEnableControl
from ..com._tmc_com import TmcCom
from .._tmc_logger import TmcLogger, Loglevel


class TmcEnableControlToff(TmcEnableControl):
    """Enable Control base class"""

    _tmc_com:TmcCom = None

    _default_toff = 3


    @property
    def tmc_com(self):
        """get the tmc_logger"""
        return self._tmc_com

    @tmc_com.setter
    def tmc_com(self, tmc_com):
        """set the tmc_logger"""
        self._tmc_com = tmc_com


    def set_motor_enabled(self, en):
        """enables or disables the motor current output

        Args:
            en (bool): whether the motor current output should be enabled
        """
        self._tmc_logger.log(f"Motor output active: {en}", Loglevel.INFO)

        val = self._default_toff if en else 0

        self._tmc_com.tmc_registers["chopconf"].modify("toff", val)
