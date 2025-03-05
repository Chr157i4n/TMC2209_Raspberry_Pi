"""
this file contains the hexadecimal addresses of the different registers
"""

from enum import Enum


class TmcRegAddr(Enum):
    """Enum for the register addresses of the TMC220x"""
    GCONF           = 0x00
    GSTAT           = 0x01
    IFCNT           = 0x02
    NODECONF        = 0x03
    IOIN            = 0x04
    DRV_CONF        = 0x0A
    GLOBAL_SCALER   = 0x0B
    IHOLD_IRUN      = 0x10
    TPOWERDOWN      = 0x11
    TSTEP           = 0x12
    TPWMTHRS        = 0x13
    TCOOLTHRS       = 0x14
    THIGH           = 0x15
    DIRECT_MODE     = 0x2D
    ENCMODE         = 0x38
    X_ENC           = 0x39
    ENC_CONST       = 0x3A
    ENC_STATUS      = 0x3B
    ENC_LATCH       = 0x3C
    ADC_VSUPPLY_AIN = 0x50
    ADC_TEMP        = 0x51
    OTW_OV_VTH      = 0x52

    MSCNT           = 0x6A
    MSCURACT        = 0x6B
    CHOPCONF        = 0x6C
    COOLCONF        = 0x6D
    DRVSTATUS       = 0x6F
    SG4_THRS        = 0x74
    SG4_RESULT      = 0x75
    SG4_IND         = 0x76
