#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
debug file for debuging the UART connection
"""

import time
try:
    from src.tmc_driver.tmc_2209 import *
except ModuleNotFoundError:
    from tmc_driver.tmc_2209 import *


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the Tmc2209 class
# use your pins for pin_en, pin_step, pin_dir here
#-----------------------------------------------------------------------
if BOARD == Board.RASPBERRY_PI:
    tmc = Tmc2209(21, 16, 20, skip_uart_init=True)
elif BOARD == Board.RASPBERRY_PI5:
    tmc = Tmc2209(21, 16, 20, serialport="/dev/ttyAMA0", skip_uart_init=True)
elif BOARD == Board.NVIDIA_JETSON:
    tmc = Tmc2209(13, 6, 5, serialport="/dev/ttyTHS1", skip_uart_init=True)
else:
    # just in case
    tmc = Tmc2209(21, 16, 20, skip_uart_init=True)




#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.tmc_logger.loglevel = Loglevel.DEBUG
tmc.movement_abs_rel = MovementAbsRel.ABSOLUTE







#-----------------------------------------------------------------------
# these functions read and print the current settings in the TMC register
#-----------------------------------------------------------------------
print("---\n---")

tmc.test_uart()


print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the Tmc2209 class
#-----------------------------------------------------------------------
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
