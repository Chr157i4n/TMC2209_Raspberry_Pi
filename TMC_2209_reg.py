#addresses
GCONF           =   0x00
GSTAT           =   0x01
IFCNT           =   0x02
DRVSTATUS       =   0x6F
CHOPCONF        =   0x6C
IOIN            =   0x06
IHOLD_IRUN      =   0x10
SG_RESULT       =   0x41


#GCONF
i_scale_analog      = 1<<0
internal_rsense     = 1<<1
en_spreadcycle      = 1<<2
shaft               = 1<<3
index_otpw          = 1<<4
index_step          = 1<<5
mstep_reg_select    = 1<<7

#CHOPCONF
vsense              = 1<<17
msres0              = 1<<24
msres1              = 1<<25
msres2              = 1<<26
msres3              = 1<<27
intpol              = 1<<28

#IOIN
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

