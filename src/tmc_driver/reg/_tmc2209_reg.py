#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
"""
Register module
"""

from ._tmc220x_reg import *



class TCoolThrs(TmcReg):
    """TCOOLTHRS register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["tcoolthrs",           0,  0xFFFFF, int]
        ]
        super().__init__(0x14, "TCOOLTHRS", tmc_com, reg_map)


class SGThrs(TmcReg):
    """SGTHRS register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["sgthrs",              0,  0xFFFFF, int]
        ]
        super().__init__(0x40, "SGTHRS", tmc_com, reg_map)


class SGResult(TmcReg):
    """SGRESULT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["sgresult",            0,  0xFFFFF, int]
        ]
        super().__init__(0x41, "SGRESULT", tmc_com, reg_map)