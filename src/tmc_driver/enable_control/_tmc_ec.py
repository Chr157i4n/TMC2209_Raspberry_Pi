"""
Enable Control base module
"""

from .._tmc_logger import TmcLogger


class TmcEnableControl():
    """Enable Control base class"""

    _tmc_logger:TmcLogger



    def init(self, tmc_logger: TmcLogger):
        """init: called by the Tmc class"""
        self._tmc_logger = tmc_logger


    def set_motor_enabled(self, en):
        """enables or disables the motor current output

        Args:
            en (bool): whether the motor current output should be enabled
        """
        raise NotImplementedError
