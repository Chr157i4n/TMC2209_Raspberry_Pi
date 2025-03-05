"""
this file contains the bitposition and bitmasks of the different values of the following register:
DRVSTATUS
"""

stst_bp         = 31
stst_bm         = 0x1

olb_bp          = 30
olb_bm          = 0x1

ola_bp          = 29
ola_bm          = 0x1

s2gb_bp         = 28
s2gb_bm         = 0x1

s2ga_bp         = 27
s2ga_bm         = 0x1

otpw_bp         = 26
otpw_bm         = 0x1

ot_bp           = 25
ot_bm           = 0x1

stallguard_bp   = 24
stallguard_bm   = 0x1

cs_actual_bp    = 16
cs_actual_bm    = 0x1F

fsactive_bp      = 15
fsactive_bm      = 0x1

stealth_bp      = 14
stealth_bm      = 0x1

s2vsb_bp        = 13
s2vsb_bm        = 0x1

s2vsa_bp        = 12
s2vsa_bm        = 0x1

sg_result_bp    = 0
sg_result_bm    = 0x3FF
