#pylint: disable=invalid-name
"""
this file contains:
1. hexadecimal address of the different registers
2. bitposition and bitmasks of the different values of each register

Example:
the register IOIN has the address 0x06 and the first bit shows
whether the ENABLE (EN/ENN) Pin is currently HIGH or LOW
"""

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
i_scale_analog_bp   = 0
i_scale_analog_bm   = 0x1

internal_rsense_bp  = 1
internal_rsense_bm  = 0x1

en_spreadcycle_bp   = 2
en_spreadcycle_bm   = 0x1

shaft_bp            = 3
shaft_bm            = 0x1

index_otpw_bp       = 4
index_otpw_bm       = 0x1

index_step_bp       = 5
index_step_bm       = 0x1

pdn_disable_bp      = 6
pdn_disable_bm      = 0x1

mstep_reg_select_bp = 7
mstep_reg_select_bm = 0x1

multistep_filt_bp   = 8
multistep_filt_bm   = 0x1

test_mode_bp        = 9
test_mode_bm        = 0x1


#GSTAT
gstat_reset_bp      = 0
gstat_reset_bm      = 0x1

gstat_drv_err_bp    = 1
gstat_drv_err_bm    = 0x1

gstat_uv_cp_bp      = 2
gstat_uv_cp_bm      = 0x1


#CHOPCONF
chopconf_diss2vs_bp = 31
chopconf_diss2vs_bm = 0x1

chopconf_diss2g_bp  = 30
chopconf_diss2g_bm  = 0x1

chopconf_dedge_bp   = 29
chopconf_dedge_bm   = 0x1

chopconf_intpol_bp  = 28
chopconf_intpol_bm  = 0x1

chopconf_mres_bp   = 24
chopconf_mres_bm   = 0xF

chopconf_vsense_bp  = 17
chopconf_vsense_bm  = 0x1

chopconf_tbl_bp     = 15
chopconf_tbl_bm     = 0x3

chopconf_hend_bp    = 7
chopconf_hend_bm    = 0xF

chopconf_hstrt_bp   = 4
chopconf_hstrt_bm   = 0x7

chopconf_toff_bp    = 0
chopconf_toff_bm    = 0xF


#IOIN
io_enn_bp           = 0
io_enn_bm           = 0x1

io_ms1_bp           = 2
io_ms1_bm           = 0x1

io_ms2_bp           = 3
io_ms2_bm           = 0x1

io_step_bp          = 7
io_step_bm          = 0x1

io_spread_bp        = 8
io_spread_bm        = 0x1

io_dir_bp           = 9
io_dir_bm           = 0x1

io_version_bp       = 24
io_version_bm       = 0xFF

#DRVSTATUS
stst_bp         = 31
stst_bm         = 0x1

stealth_bp      = 30
stealth_bm      = 0x1

cs_actual_bp    = 16
cs_actual_bm    = 0x1F

t157_bp         = 11
t157_bm         = 0x1

t150_bp         = 10
t150_bm         = 0x1

t143_bp         = 9
t143_bm         = 0x1

t120_bp         = 8
t120_bm         = 0x1

olb_bp          = 7
olb_bm          = 0x1

ola_bp          = 6
ola_bm          = 0x1

s2vsb_bp        = 5
s2vsb_bm        = 0x1

s2vsa_bp        = 4
s2vsa_bm        = 0x1

s2gb_bp         = 3
s2gb_bm         = 0x1

s2ga_bp         = 2
s2ga_bm         = 0x1

ot_bp           = 1
ot_bm           = 0x1

otpw_bp         = 0
otpw_bm         = 0x1


#IHOLD_IRUN
ihold_irun_ihold_bp = 0
ihold_irun_ihold_bm = 0x1F

ihold_irun_irun_bp  = 8
ihold_irun_irun_bm  = 0x1F

ihold_irun_iholddelay_bp = 16
ihold_irun_iholddelay_bm = 0xF
