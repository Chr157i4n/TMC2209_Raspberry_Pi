#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=attribute-defined-outside-init
"""
Register module
"""

from ._tmc220x_reg import *



class TCoolThrs(TmcReg):
    """TCOOLTHRS register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["tcoolthrs",           0,  0xFFFFF, int, None, ""]
        ]
        super().__init__(0x14, "TCOOLTHRS", tmc_com, reg_map)


class SGThrs(TmcReg):
    """SGTHRS register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["sgthrs",              0,  0xFFFFF, int, None, ""]
        ]
        super().__init__(0x40, "SGTHRS", tmc_com, reg_map)


class SGResult(TmcReg):
    """SGRESULT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["sgresult",            0,  0xFFFFF, int, None, ""]
        ]
        super().__init__(0x41, "SGRESULT", tmc_com, reg_map)


class CoolConf(TmcReg):
    """COOLCONF register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["seemin",              15, 0x1,    bool,   None, ""],
            ["sedn",                13, 0x3,    int,    None, ""],
            ["semax",               8, 0xF,     int,    None, ""],
            ["seup",                5, 0x3,     int,    None, ""],
            ["semin",               0, 0xF,     int,    None, ""]
        ]
        super().__init__(0x42, "COOLCONF", tmc_com, reg_map)
