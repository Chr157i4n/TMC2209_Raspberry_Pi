"""
Enable Control base module
"""

from ._tmc_ec import TmcEnableControl
from ..com._tmc_com import TmcCom
from ..reg.tmc220x._tmc_chopconf import ChopConf


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
        chopconf = ChopConf()
        chopconf.read(self._tmc_com)

        if en:
            chopconf.toff = self._default_toff
        else:
            chopconf.toff = 0

        chopconf.write(self._tmc_com)
