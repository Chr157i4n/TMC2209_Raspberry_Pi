"""
this file contains the bitposition and bitmasks of the different values of the following register:
GCONF
"""

direct_mode_bp = 16
direct_mode_bm = 0x1

stop_enable_bp = 15
stop_enable_bm = 0x1

small_hysteresis_bp = 14
small_hysteresis_bm = 0x1

diag1_pushpull_bp = 13
diag1_pushpull_bm = 0x1

diag0_pushpull_bp = 12
diag0_pushpull_bm = 0x1

diag1_onstate_bp = 10
diag1_onstate_bm = 0x1

diag1_index_bm = 0x1
diag1_index_bp = 9

diag1_stall_bp = 8
diag1_stall_bm = 0x1

diag0_stall_bp = 7
diag0_stall_bm = 0x1

diag0_otpw_bp = 6
diag0_otpw_bm = 0x1

diag0_error_bp = 5
diag0_error_bm = 0x1

shaft_bp = 4
shaft_bm = 0x1

multistep_filt_bp = 3
multistep_filt_bm = 0x1

en_pwm_mode_bp = 2
en_pwm_mode_bm = 0x1

fast_standstill_bp = 1
fast_standstill_bm = 0x1
