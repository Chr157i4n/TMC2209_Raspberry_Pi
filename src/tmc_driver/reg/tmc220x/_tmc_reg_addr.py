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
    OTP_PROG        = 0x04
    OTP_READ        = 0x05
    IOIN            = 0x06
    FACTORY_CONF    = 0x07
    IHOLD_IRUN      = 0x10
    TPOWERDOWN      = 0x11
    TSTEP           = 0x12
    TPWMTHRS        = 0x13
    TCOOLTHRS       = 0x14
    VACTUAL         = 0x22
    SGTHRS          = 0x40
    SG_RESULT       = 0x41
    COOLCONF        = 0x42
    MSCNT           = 0x6A
    MSCURACT        = 0x6B
    CHOPCONF        = 0x6C
    DRVSTATUS       = 0x6F
    PWMCONF         = 0x70
    PWM_SCALE       = 0x71
    PWM_AUTO        = 0x72
