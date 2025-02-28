#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
test file for testing writing the log messages to a file
"""

import logging
try:
    from src.tmc_driver.tmc_2209 import *
    from src.tmc_driver._tmc_gpio_board import Board
except ModuleNotFoundError:
    from tmc_driver.tmc_2209 import *
    from tmc_driver._tmc_gpio_board import Board



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
# initiate the TMC_2209 class
# use your pins for pin_en, pin_step, pin_dir here
#-----------------------------------------------------------------------
if BOARD == Board.RASPBERRY_PI:
    tmc = Tmc2209(21, 16, 20, skip_uart_init=True,
                   loglevel=loglevel, log_handlers=[logging_handler], log_formatter=logformatter)
elif BOARD == Board.RASPBERRY_PI5:
    tmc = Tmc2209(21, 16, 20, serialport="/dev/ttyAMA0", skip_uart_init=True,
                   loglevel=loglevel, log_handlers=[logging_handler], log_formatter=logformatter)
elif BOARD == Board.NVIDIA_JETSON:
    tmc = Tmc2209(13, 6, 5, serialport="/dev/ttyTHS1", skip_uart_init=True,
                   loglevel=loglevel, log_handlers=[logging_handler], log_formatter=logformatter)
else:
    # just in case
    tmc = Tmc2209(21, 16, 20, skip_uart_init=True,
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
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
