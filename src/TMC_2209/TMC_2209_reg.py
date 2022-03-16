#-----------------------------------------------------------------------
# this file contains:
# 1. hexadecimal address of the different registers
# 2. bitposition and bitmasks of the different values of each register
#
# Example:
# the register IOIN has the address 0x06 and the first bit shows
# whether the ENABLE (EN/ENN) Pin is currently HIGH or LOW
#-----------------------------------------------------------------------

#addresses
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

#GCONF
i_scale_analog      = 1<<0
internal_rsense     = 1<<1
en_spreadcycle      = 1<<2
shaft               = 1<<3
index_otpw          = 1<<4
index_step          = 1<<5
mstep_reg_select    = 1<<7

#GSTAT
reset               = 1<<0
drv_err             = 1<<1
uv_cp               = 1<<2

#CHOPCONF
vsense              = 1<<17
msres0              = 1<<24
msres1              = 1<<25
msres2              = 1<<26
msres3              = 1<<27
intpol              = 1<<28

#IOIN
io_enn              = 1<<0
io_step             = 1<<7
io_spread           = 1<<8
io_dir              = 1<<9

#DRVSTATUS
stst                = 1<<31
stealth             = 1<<30
cs_actual           = 31<<16
t157                = 1<<11
t150                = 1<<10
t143                = 1<<9
t120                = 1<<8
olb                 = 1<<7
ola                 = 1<<6
s2vsb               = 1<<5
s2vsa               = 1<<4
s2gb                = 1<<3
s2ga                = 1<<2
ot                  = 1<<1
otpw                = 1<<0

#IHOLD_IRUN
ihold               = 31<<0
irun                = 31<<8
iholddelay          = 15<<16

#SGTHRS
sgthrs              = 255<<0

#others
mres_256 = 0
mres_128 = 1
mres_64 = 2
mres_32 = 3
mres_16 = 4
mres_8 = 5
mres_4 = 6
mres_2 = 7
mres_1 = 8

