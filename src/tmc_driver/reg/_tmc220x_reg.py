#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=attribute-defined-outside-init
"""
Register module
"""

import math
from ._tmc_reg import *


class GConf(TmcReg):
    """GCONF register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["test_mode",           9, 0x1, bool],
            ["multistep_filt",      8, 0x1, bool],
            ["mstep_reg_select",    7, 0x1, bool],
            ["pdn_disable",         6, 0x1, bool],
            ["index_step",          5, 0x1, bool],
            ["index_otpw",          4, 0x1, bool],
            ["shaft",               3, 0x1, bool],
            ["en_spreadcycle",      2, 0x1, bool],
            ["internal_rsense",     1, 0x1, bool],
            ["i_scale_analog",      0, 0x1, bool]
        ]
        super().__init__(0x0, "GCONF", tmc_com, reg_map)


class GStat(TmcReg):
    """GSTAT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["uv_cp",               2, 0x1, bool],
            ["drv_err",             1, 0x1, bool],
            ["reset",               0, 0x1, bool]
        ]
        super().__init__(0x1, "GSTAT", tmc_com, reg_map)


class IfCnt(TmcReg):
    """IFCNT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["ifcnt",               0, 0xFF, int]
        ]
        super().__init__(0x2, "IFCNT", tmc_com, reg_map)


class Ioin(TmcReg):
    """IOIN register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["version",             24, 0xFF, int],
            ["dir",                 9,  0x1, bool],
            ["spread",              8,  0x1, bool],
            ["step",                7,  0x1, bool],
            ["ms2",                 3,  0x1, bool],
            ["ms1",                 2,  0x1, bool],
            ["enn",                 0,  0x1, bool]
        ]
        super().__init__(0x3, "IOIN", tmc_com, reg_map)


class IHoldIRun(TmcReg):
    """IHOLD_IRUN register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["iholddelay",          16, 0xF, int],
            ["irun",                8,  0x1F, int],
            ["ihold",               0,  0x1F, int]
        ]
        super().__init__(0x10, "IHOLD_IRUN", tmc_com, reg_map)


class TPowerDown(TmcReg):
    """TPowerDown register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["tpowerdown",          0,  0xFF, int]
        ]
        super().__init__(0x11, "TPowerDown", tmc_com, reg_map)


class TStep(TmcReg):
    """TSTEP register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["tstep",               0,  0xFFFFF, int]
        ]
        super().__init__(0x12, "TSTEP", tmc_com, reg_map)


class VActual(TmcReg):
    """VACTUAL register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["vactual",             0,  0x1FFFFF, int]
        ]
        super().__init__(0x22, "VACTUAL", tmc_com, reg_map)


class MsCnt(TmcReg):
    """MSCNT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["mscnt",               0,  0xFF, int]
        ]
        super().__init__(0x6A, "MSCNT", tmc_com, reg_map)


class ChopConf(TmcReg):
    """CHOPCONF register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["diss2vs",             31, 0x1, bool],
            ["diss2g",              30, 0x1, bool],
            ["dedge",               29, 0x1, bool],
            ["intpol",              28, 0x1, bool],
            ["mres",                24, 0xF, int],
            ["vsense",              17, 0x1, bool],
            ["tbl",                 15, 0x3, int],
            ["hend",                7,  0xF, int],
            ["hstrt",               4,  0x7, int],
            ["toff",                0,  0xF, int]
        ]
        super().__init__(0x6C, "CHOPCONF", tmc_com, reg_map)


    def convert_mres_to_reg(self, mres: int):
        """converts the µstep resolution to the corresponding register value

        Args:
            mres (int): µstep resolution
        """
        mres_bit = int(math.log(mres, 2))
        mres_bit = 8 - mres_bit
        self.mres = mres_bit


    def convert_reg_to_mres(self) -> int:
        """converts the register value to the corresponding µstep resolution

        Returns:
            int: µstep resolution
        """
        return int(math.pow(2, 8 - self.mres))


class DrvStatus(TmcReg):
    """DRVSTATUS register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["stst",                31, 0x1, bool],
            ["stealth",             30, 0x1, bool],
            ["cs_actual",           16, 0x1F, int],
            ["t157",                11, 0x1, bool],
            ["t150",                10, 0x1, bool],
            ["t143",                9,  0x1, bool],
            ["t120",                8,  0x1, bool],
            ["olb",                 7,  0x1, bool],
            ["ola",                 6,  0x1, bool],
            ["s2vsb",               5,  0x1, bool],
            ["s2vsa",               4,  0x1, bool],
            ["s2gb",                3,  0x1, bool],
            ["s2ga",                2,  0x1, bool],
            ["ot",                  1,  0x1, bool],
            ["otpw",                0,  0x1, bool]
        ]
        super().__init__(0x6F, "DRVSTATUS", tmc_com, reg_map)
