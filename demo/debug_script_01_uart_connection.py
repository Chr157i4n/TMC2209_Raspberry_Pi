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
    tmc = Tmc2209(21, 16, 20, None)
elif BOARD == Board.RASPBERRY_PI5:
    tmc = Tmc2209(21, 16, 20, None)
elif BOARD == Board.NVIDIA_JETSON:
    tmc = Tmc2209(13, 6, 5, None)
else:
    # just in case
    tmc = Tmc2209(21, 16, 20)


if BOARD == Board.RASPBERRY_PI:
    tmc.tmc_com = TmcUart("/dev/serial0")
elif BOARD == Board.RASPBERRY_PI5:
    tmc.tmc_com = TmcUart("/dev/ttyAMA0")
elif BOARD == Board.NVIDIA_JETSON:
    tmc.tmc_com = TmcUart("/dev/ttyTHS1")

tmc.tmc_com.tmc_logger = tmc.tmc_logger


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

tmc.test_com()


print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the Tmc2209 class
#-----------------------------------------------------------------------
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
