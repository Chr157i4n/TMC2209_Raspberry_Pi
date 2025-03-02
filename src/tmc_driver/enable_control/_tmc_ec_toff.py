"""
Enable Control base module
"""

from ._tmc_ec import TmcEnableControl


class TmcEnableControlToff(TmcEnableControl):
    """Enable Control base class"""


    def set_motor_enabled(self, en):
        """enables or disables the motor current output

        Args:
            en (bool): whether the motor current output should be enabled
        """
        raise NotImplementedError
