"""
test file for testing writing the log messages to a file
"""

import logging
try:
    from src.tmc_driver.tmc_2209 import *
except ModuleNotFoundError:
    from tmc_driver.tmc_2209 import *


print("---")
print("SCRIPT START")
print("---")


#-----------------------------------------------------------------------
# initiate the log level, handler, and formatter
#-----------------------------------------------------------------------
loglevel = Loglevel.ALL
logging_handler = logging.FileHandler("tmc2209_log_file.log")
logformatter = logging.Formatter('%(name)s %(asctime)s - %(levelname)s - %(message)s',
                                 '%Y%m%d %H:%M:%S')






#-----------------------------------------------------------------------
# initiate the Tmc2209 class
# use your pins for pin_en, pin_step, pin_dir here
#-----------------------------------------------------------------------
if BOARD == Board.RASPBERRY_PI:
    tmc = Tmc2209(TmcEnableControlPin(21), TmcMotionControlStepDir(16, 20), TmcComUart("/dev/serial0"),
                   loglevel=loglevel, log_handlers=[logging_handler], log_formatter=logformatter)
elif BOARD == Board.RASPBERRY_PI5:
    tmc = Tmc2209(TmcEnableControlPin(21), TmcMotionControlStepDir(16, 20), TmcComUart("/dev/ttyAMA0"),
                   loglevel=loglevel, log_handlers=[logging_handler], log_formatter=logformatter)
elif BOARD == Board.NVIDIA_JETSON:
    tmc = Tmc2209(TmcEnableControlPin(13), TmcMotionControlStepDir(6, 5), TmcComUart("/dev/ttyTHS1"),
                   loglevel=loglevel, log_handlers=[logging_handler], log_formatter=logformatter)
else:
    # just in case
    tmc = Tmc2209(TmcEnableControlPin(21), TmcMotionControlStepDir(16, 20), TmcComUart("/dev/serial0"),
                   loglevel=loglevel, log_handlers=[logging_handler], log_formatter=logformatter)





#-----------------------------------------------------------------------
# Log custom messages
#-----------------------------------------------------------------------
tmc.tmc_logger.log("========================", Loglevel.ALL)
tmc.tmc_logger.log("Hello World!", Loglevel.DEBUG)
tmc.tmc_logger.log("Wow, you can even log your own messages!", Loglevel.ERROR)
tmc.tmc_logger.log("If you like this library, please give us a star on GitHub!", Loglevel.INFO)
tmc.tmc_logger.log("The cake is a lie", Loglevel.WARNING)
tmc.tmc_logger.log("I like to move it, move it", Loglevel.MOVEMENT)
tmc.tmc_logger.log("========================", Loglevel.ALL)





#-----------------------------------------------------------------------
# deinitiate the Tmc2209 class
#-----------------------------------------------------------------------
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
