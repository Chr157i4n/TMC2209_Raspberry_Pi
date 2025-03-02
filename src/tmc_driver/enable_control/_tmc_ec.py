"""
Enable Control base module
"""

from .._tmc_logger import TmcLogger, Loglevel


class TmcEnableControl():
    """Enable Control base class"""

    _tmc_logger:TmcLogger

    @property
    def tmc_logger(self):
        """_tmc_logger property"""
        return self._tmc_logger

    @tmc_logger.setter
    def tmc_logger(self, tmc_logger:TmcLogger):
        """_tmc_logger setter"""
        self._tmc_logger = tmc_logger



    def init(self):
        """init: called by the Tmc class"""
        pass


    def set_motor_enabled(self, en):
        """enables or disables the motor current output

        Args:
            en (bool): whether the motor current output should be enabled
        """
        raise NotImplementedError
