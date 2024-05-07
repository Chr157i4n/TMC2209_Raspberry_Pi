#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
#pylint: disable=broad-exception-raised
"""
test file for testing multiple drivers via one UART connection
"""

import time
try:
    from src.TMC_2209.TMC_2209_StepperDriver import *
except ModuleNotFoundError:
    from TMC_2209.TMC_2209_StepperDriver import *


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_en, pin_step, pin_dir here
#-----------------------------------------------------------------------
# Multiple driver not tested
if BOARD == "RASPBERRY_PI":
    tmc1 = TMC_2209(21, 16, 20, driver_address=0)
    tmc2 = TMC_2209(26, 13, 19, driver_address=1)
elif BOARD == "NVIDIA_JETSON":
    # tmc1 = TMC_2209(13, 6, 5, serialport="/dev/ttyTHS1", driver_address=0)
    raise Exception("Not tested for Nvidia Jetson, use with caution")
else:
    # just in case
    tmc1 = TMC_2209(21, 16, 20, driver_address=0)
    tmc2 = TMC_2209(26, 13, 19, driver_address=1)








#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc1.tmc_logger.set_loglevel(Loglevel.DEBUG)
tmc1.set_movement_abs_rel(MovementAbsRel.ABSOLUTE)

tmc2.tmc_logger.set_loglevel(Loglevel.DEBUG)
tmc2.set_movement_abs_rel(MovementAbsRel.ABSOLUTE)



#-----------------------------------------------------------------------
# these functions read and print the current settings in the TMC register
#-----------------------------------------------------------------------

print("---")
print("IOIN tmc1")
print("---")
tmc1.read_ioin()

print("---\n---")


print("---")
print("IOIN tmc2")
print("---")
tmc2.read_ioin()

print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
tmc1.set_motor_enabled(False)
tmc2.set_motor_enabled(False)
del tmc1
del tmc2

print("---")
print("SCRIPT FINISHED")
print("---")
