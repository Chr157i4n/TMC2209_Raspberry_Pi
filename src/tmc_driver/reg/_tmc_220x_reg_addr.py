"""
this file contains the hexadecimal addresses of the different registers
"""

from enum import Enum


class TmcRegAddr(Enum):
    """Enum for the register addresses of the TMC220x"""
    GCONF           =   0x00
    GSTAT           =   0x01
    IFCNT           =   0x02
    IOIN            =   0x06
    IHOLD_IRUN      =   0x10
    TSTEP           =   0x12
    VACTUAL         =   0x22
    TCOOLTHRS       =   0x14
    SGTHRS          =   0x40
    SG_RESULT       =   0x41
    MSCNT           =   0x6A
    CHOPCONF        =   0x6C
    DRVSTATUS       =   0x6F
