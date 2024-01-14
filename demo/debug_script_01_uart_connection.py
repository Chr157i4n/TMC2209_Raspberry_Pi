#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
debug file for debuging the UART connection
"""

import time
from src.TMC_2209.TMC_2209_StepperDriver import *


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_en, pin_step, pin_dir here
#-----------------------------------------------------------------------
if BOARD == "RASPBERRY_PI":
    tmc = TMC_2209(21, 16, 20, skip_uart_init=True)
elif BOARD == "NVIDIA_JETSON":
    tmc = TMC_2209(13, 6, 5, serialport="/dev/ttyTHS1", skip_uart_init=True)
else:
    # just in case
    tmc = TMC_2209(21, 16, 20, skip_uart_init=True)




#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.tmc_logger.set_loglevel(Loglevel.DEBUG)
tmc.set_movement_abs_rel(MovementAbsRel.ABSOLUTE)







#-----------------------------------------------------------------------
# these functions read and print the current settings in the TMC register
#-----------------------------------------------------------------------
print("---\n---")

tmc.test_uart()


print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
