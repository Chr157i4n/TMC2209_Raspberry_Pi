#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=attribute-defined-outside-init
#pylint: disable=no-member
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
            ["direct_mode",         16, 0x1, bool, None, ""],
            ["stop_enable",         15, 0x1, bool, None, ""],
            ["small_hysteresis",    14, 0x1, bool, None, ""],
            ["diag1_pushpull",      13, 0x1, bool, None, ""],
            ["diag0_pushpull",      12, 0x1, bool, None, ""],
            ["diag1_onstate",       10, 0x1, bool, None, ""],
            ["diag1_index",         9,  0x1, bool, None, ""],
            ["diag1_stall",         8,  0x1, bool, None, ""],
            ["diag0_stall",         7,  0x1, bool, None, ""],
            ["diag0_otpw",          6,  0x1, bool, None, ""],
            ["diag0_error",         5,  0x1, bool, None, ""],
            ["shaft",               4,  0x1, bool, None, ""],
            ["multistep_filt",      3,  0x1, bool, None, ""],
            ["en_pwm_mode",         2,  0x1, bool, None, ""],
            ["fast_standstill",     1,  0x1, bool, None, ""]
        ]
        super().__init__(0x0, "GCONF", tmc_com, reg_map)


class GStat(TmcReg):
    """GSTAT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["vm_uvlo",             4,  0x1, bool, None, ""],
            ["register_reset",      3,  0x1, bool, None, ""],
            ["uv_cp",               2,  0x1, bool, None, ""],
            ["drv_err",             1,  0x1, bool, None, ""],
            ["reset",               0,  0x1, bool, None, ""]
        ]
        super().__init__(0x1, "GSTAT", tmc_com, reg_map)


class IfCnt(TmcReg):
    """IFCNT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["ifcnt",               0, 0xFF, int, None, ""]
        ]
        super().__init__(0x2, "IFCNT", tmc_com, reg_map)


class Ioin(TmcReg):
    """IOIN register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["version",             24, 0xFF, int, None, ""],
            ["silicon_rv",          24, 0xFF, int, None, ""],
            ["adc_err",             15, 0x1, bool, None, ""],
            ["ext_clk",             14, 0x1, bool, None, ""],
            ["ext_res_det",         13, 0x1, bool, None, ""],
            ["output",              12, 0x1, bool, None, ""],
            ["comp_b1_b2",          11, 0x1, bool, None, ""],
            ["comp_a1_a2",          10, 0x1, bool, None, ""],
            ["comp_b",              9,  0x1, bool, None, ""],
            ["comp_a",              8,  0x1, bool, None, ""],
            ["uart_en",             6,  0x1, bool, None, ""],
            ["encn",                5,  0x1, bool, None, ""],
            ["drv_enn",             4,  0x1, bool, None, ""],
            ["enca",                3,  0x1, bool, None, ""],
            ["encb",                2,  0x1, bool, None, ""],
            ["dir",                 1,  0x1, bool, None, ""],
            ["step",                0,  0x1, bool, None, ""]
        ]
        super().__init__(0x4, "IOIN", tmc_com, reg_map)


class DrvConf(TmcReg):
    """DRV_CONF register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["slope_control",       4, 0x3, int, None, ""],
            ["current_range",       0, 0x3, int, None, ""],
        ]
        super().__init__(0x6, "DRV_CONF", tmc_com, reg_map)


class GlobalScaler(TmcReg):
    """GLOBAL_SCALER register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["global_scaler",       0, 0xFF, int, None, ""]
        ]
        super().__init__(0xB, "GLOBAL_SCALER", tmc_com, reg_map)


class IHoldIRun(TmcReg):
    """IHOLD_IRUN register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["irundelay",           24, 0xF, int, None, ""],
            ["iholddelay",          16, 0xF, int, None, ""],
            ["irun",                8,  0x1F, int, None, ""],
            ["ihold",               0,  0x1F, int, None, ""]
        ]
        super().__init__(0x10, "IHOLD_IRUN", tmc_com, reg_map)


class TPowerDown(TmcReg):
    """TPOWERDOWN register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["tpowerdown",          0, 0xFF, int, None, ""]
        ]
        super().__init__(0x11, "TPOWERDOWN", tmc_com, reg_map)


class TStep(TmcReg):
    """TSTEP register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["tstep",               0, 0xFFFFF, int, None, ""]
        ]
        super().__init__(0x12, "TSTEP", tmc_com, reg_map)


class ADCVSupplyAIN(TmcReg):
    """ADCV_SUPPLY_AIN register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["adc_ain",             16, 0xFFFF, int, lambda: self.adc_ain_v, "V"],
            ["adc_vsupply",         0,  0xFFFF, int, lambda: self.adc_vsupply_v, "V"]
        ]
        super().__init__(0x50, "ADCV_SUPPLY_AIN", tmc_com, reg_map)

    @property
    def adc_vsupply_v(self) -> float:
        """return Supplyvoltage in V"""
        if hasattr(self, "adc_vsupply"):
            return round(self.adc_vsupply * 9.732/1000, 2)
        return None

    @property
    def adc_ain_v(self) -> float:
        """return voltage on AIN in V"""
        if hasattr(self, "adc_ain"):
            return round(self.adc_ain * 305.2/1000/1000, 2)
        return None


class ADCTemp(TmcReg):
    """ADC_TEMP register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["adc_temp",            0,  0xFFFF, int, lambda: self.adc_temp_c, "°C"]
        ]
        super().__init__(0x51, "ADC_TEMP", tmc_com, reg_map)

    @property
    def adc_temp_c(self) -> float:
        """return temperature in °C"""
        if hasattr(self, "adc_temp"):
            return round((self.adc_temp - 2038) / 7.7, 1)
        return None


class ChopConf(TmcReg):
    """CHOPCONF register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["diss2vs",             31, 0x1, bool, None, ""],
            ["diss2g",              30, 0x1, bool, None, ""],
            ["dedge",               29, 0x1, bool, None, ""],
            ["intpol",              28, 0x1, bool, None, ""],
            ["mres",                24, 0xF, int, lambda: self.mres_ms, "µStep"],
            ["tpfd",                20, 0xF, int, None, ""],
            ["vhighchm",            19, 0x1, bool, None, ""],
            ["vhighfs",             18, 0x1, bool, None, ""],
            ["tbl",                 15, 0x3, int, None, ""],
            ["chm",                 14, 0x3, int, None, ""],
            ["disfdcc",             12, 0x1, bool, None, ""],
            ["fd3",                 11, 0x1, bool, None, ""],
            ["hend_offset",         7,  0xF, int, None, ""],
            ["hstrt_tfd210",        4,  0x7, int, None, ""],
            ["toff",                0,  0xF, int, None, ""]
        ]
        super().__init__(0x6C, "CHOPCONF", tmc_com, reg_map)

    @property
    def mres_ms(self) -> int:
        """return µstep resolution"""
        if hasattr(self, "mres"):
            return int(math.pow(2, 8 - self.mres))
        return None

    @mres_ms.setter
    def mres_ms(self, mres: int):
        """set µstep resolution"""
        mres_bit = int(math.log(mres, 2))
        mres_bit = 8 - mres_bit
        self.mres = mres_bit


class DrvStatus(TmcReg):
    """DRVSTATUS register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["stst",                31, 0x1, bool, None, ""],
            ["olb",                 30, 0x1, bool, None, ""],
            ["ola",                 29, 0x1, bool, None, ""],
            ["s2gb",                28, 0x1, bool, None, ""],
            ["s2ga",                27, 0x1, bool, None, ""],
            ["otpw",                26, 0x1, bool, None, ""],
            ["ot",                  25, 0x1, bool, None, ""],
            ["stallguard",          24, 0x1, bool, None, ""],
            ["cs_actual",           16, 0x1F, int, None, ""],
            ["fsactive",            15, 0x1, bool, None, ""],
            ["stealth",             14, 0x1, bool, None, ""],
            ["s2vsb",               13, 0x1, bool, None, ""],
            ["s2vsa",               12, 0x1, bool, None, ""],
            ["sg_result",           0,  0x3FF, int, None, ""]
        ]
        super().__init__(0x6F, "DRVSTATUS", tmc_com, reg_map)


class TCoolThrs(TmcReg):
    """TCOOLTHRS register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["tcoolthrs",           0,  0xFFFFF, int, None, ""]
        ]
        super().__init__(0x14, "TCOOLTHRS", tmc_com, reg_map)


class SgThrs(TmcReg):
    """SGTHRS register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["sg_angle_offset",      9,  0x1, bool, None, ""],
            ["sg4_filt_en",          8,  0x1, bool, None, ""],
            ["sg_thrs",              0,  0xFFF, int, None, ""]
        ]
        super().__init__(0x74, "SG_THRS", tmc_com, reg_map)


class SgResult(TmcReg):
    """SGRESULT register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["sg_result",            0,  0xFFFFF, int, None, ""]
        ]
        super().__init__(0x75, "SG_RESULT", tmc_com, reg_map)


class SgInd(TmcReg):
    """SGIND register class"""

    def __init__(self, tmc_com: TmcCom):
        """constructor"""

        reg_map = [
            ["sg_ind_2",            16, 0xFF, int, None, ""],
            ["sg_ind_1",            8, 0xFF, int, None, ""],
            ["sg_ind_0",            0,  0xFF, int, None, ""],
        ]
        super().__init__(0x76, "SG_IND", tmc_com, reg_map)