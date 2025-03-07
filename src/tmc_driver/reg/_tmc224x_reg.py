#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
"""
Register module
"""

from ._tmc_reg import *



class GConf(TmcReg):
    """GCONF register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["direct_mode",         16, 0x1, bool],
            ["stop_enable",         15, 0x1, bool],
            ["small_hysteresis",    14, 0x1, bool],
            ["diag1_pushpull",      13, 0x1, bool],
            ["diag0_pushpull",      12, 0x1, bool],
            ["diag1_onstate",       10, 0x1, bool],
            ["diag1_index",         9,  0x1, bool],
            ["diag1_stall",         8,  0x1, bool],
            ["diag0_stall",         7,  0x1, bool],
            ["diag0_otpw",          6,  0x1, bool],
            ["diag0_error",         5,  0x1, bool],
            ["shaft",               4,  0x1, bool],
            ["multistep_filt",      3,  0x1, bool],
            ["en_pwm_mode",         2,  0x1, bool],
            ["fast_standstill",     1,  0x1, bool]
        ]
        super().__init__(0x0, "GCONF", tmc_com, reg_map)


class GStat(TmcReg):
    """GSTAT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["vm_uvlo",             4,  0x1, bool],
            ["register_reset",      3,  0x1, bool],
            ["uv_cp",               2,  0x1, bool],
            ["drv_err",             1,  0x1, bool],
            ["reset",               0,  0x1, bool]
        ]
        super().__init__(0x1, "GSTAT", tmc_com, reg_map)


class IfCnt(TmcReg):
    """IFCNT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["ifcnt",               0, 0xFF, int]
        ]
        super().__init(0x2, "IFCNT", tmc_com, reg_map)


class Ioin(TmcReg):
    """IOIN register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["version",             24, 0xFF, int],
            ["silicon_rv",          24, 0xFF, int],
            ["adc_err",             15, 0x1, bool],
            ["ext_clk",             14, 0x1, bool],
            ["ext_res_det",         13, 0x1, bool],
            ["output",              12, 0x1, bool],
            ["comp_b1_b2",          11, 0x1, bool],
            ["comp_a1_a2",          10, 0x1, bool],
            ["comp_b",              9,  0x1, bool],
            ["comp_a",              8,  0x1, bool],
            ["uart_en",             6,  0x1, bool],
            ["encn",                5,  0x1, bool],
            ["drv_enn",             4,  0x1, bool],
            ["enca",                3,  0x1, bool],
            ["encb",                2,  0x1, bool],
            ["dir",                 1,  0x1, bool],
            ["step",                0,  0x1, bool]
        ]
        super().__init(0x4, "IOIN", tmc_com, reg_map)


class IHoldIRun(TmcReg):
    """IHOLD_IRUN register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["irundelay",           24, 0xF, int],
            ["iholddelay",          16, 0xF, int],
            ["irun",                8,  0x1F, int],
            ["ihold",               0,  0x1F, int]
        ]
        super().__init(0x10, "IHOLD_IRUN", tmc_com, reg_map)


class ADCVSupplyAIN(TmcReg):
    """ADCV_SUPPLY_AIN register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["adc_ain",             16, 0xFFFF, int],
            ["adc_vsupply",         0,  0xFFFF, int]
        ]
        super().__init(0x50, "ADCV_SUPPLY_AIN", tmc_com, reg_map)


class ADCTemp(TmcReg):
    """ADC_TEMP register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["adc_temp",            0,  0xFFFF, int]
        ]
        super().__init(0x51, "ADC_TEMP", tmc_com, reg_map)


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
            ["tpfd",                20, 0xF, int],
            ["vhighchm",            19, 0x1, bool],
            ["vhighfs",             18, 0x1, bool],
            ["tbl",                 15, 0x3, int],
            ["chm",                 14, 0x3, int],
            ["disfdcc",             12, 0x1, bool],
            ["fd3",                 11, 0x1, bool],
            ["hend_offset",         7,  0xF, int],
            ["hstrt_tfd210",        4,  0x7, int],
            ["toff",                0,  0xF, int]
        ]
        super().__init__(0x6C, "CHOPCONF", tmc_com, reg_map)


class DrvStatus(TmcReg):
    """DRVSTATUS register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["stst",                31, 0x1, bool],
            ["olb",                 30, 0x1, bool],
            ["ola",                 29, 0x1, bool],
            ["s2gb",                28, 0x1, bool],
            ["s2ga",                27, 0x1, bool],
            ["otpw",                26, 0x1, bool],
            ["ot",                  25, 0x1, bool],
            ["stallguard",          24, 0x1, bool],
            ["cs_actual",           16, 0x1F, int],
            ["fsactive",            15, 0x1, bool],
            ["stealth",             14, 0x1, bool],
            ["s2vsb",               13, 0x1, bool],
            ["s2vsa",               12, 0x1, bool],
            ["sg_result",           0,  0x3FF, int]
        ]
        super().__init(0x6F, "DRVSTATUS", tmc_com, reg_map)
