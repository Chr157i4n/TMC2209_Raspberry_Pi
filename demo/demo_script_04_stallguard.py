#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
test file for testing the StallGuard feature
"""

import time
try:
    from src.TMC_2209.TMC_2209_StepperDriver import *
    from src.TMC_2209._TMC_2209_GPIO_board import Board
except ModuleNotFoundError:
    from TMC_2209.TMC_2209_StepperDriver import *
    from TMC_2209._TMC_2209_GPIO_board import Board


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_en, pin_step, pin_dir here
#-----------------------------------------------------------------------
if BOARD == Board.NVIDIA_JETSON:
    raise NotImplementedError('''
        Not implemented. Needs refinement.\n
        Nvidia Jetson has nuances with the parameter pull_up_down for pin_stallguard:
        https://github.com/NVIDIA/jetson-gpio/issues/5''')
elif BOARD == Board.RASPBERRY_PI:
    tmc = TMC_2209(21, 16, 20)
elif BOARD == Board.RASPBERRY_PI5:
    tmc = TMC_2209(21, 16, 20, serialport="/dev/ttyAMA0")
else:
    # just in case
    tmc = TMC_2209(21, 16, 20)





#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.tmc_logger.set_loglevel(Loglevel.DEBUG)
tmc.set_movement_abs_rel(MovementAbsRel.ABSOLUTE)





#-----------------------------------------------------------------------
# these functions change settings in the TMC register
#-----------------------------------------------------------------------
tmc.set_direction_reg(False)
tmc.set_current(300)
tmc.set_interpolation(True)
tmc.set_spreadcycle(False)
tmc.set_microstepping_resolution(2)
tmc.set_internal_rsense(False)


print("---\n---")





#-----------------------------------------------------------------------
# these functions read and print the current settings in the TMC register
#-----------------------------------------------------------------------
tmc.read_ioin()
tmc.read_chopconf()
tmc.read_drv_status()
tmc.read_gconf()

print("---\n---")





#-----------------------------------------------------------------------
# set the Accerleration and maximal Speed
#-----------------------------------------------------------------------
tmc.set_acceleration(2000)
tmc.set_max_speed(500)





#-----------------------------------------------------------------------
# activate the motor current output
#-----------------------------------------------------------------------
tmc.set_motor_enabled(True)





#-----------------------------------------------------------------------
# runs the motor 800 steps in a thread and
# prints the stallguard result for each movement phase
#-----------------------------------------------------------------------
tmc.test_stallguard_threshold(800)





#-----------------------------------------------------------------------
# set a callback function for the stallguard interrupt based detection
# 1. param: pin connected to the tmc DIAG output
# 2. param: is the threshold StallGuard
# 3. param: is the callback function (threaded)
# 4. param (optional): min speed threshold (in steptime measured  in  clock  cycles)
#-----------------------------------------------------------------------
def my_callback():
    """StallGuard callback"""
    print("StallGuard!")
    tmc.stop()

tmc.set_stallguard_callback(26, 50, my_callback) # after this function call, StallGuard is active


#uses STEP/DIR to move the motor
finishedsuccessfully = tmc.run_to_position_steps(4000, MovementAbsRel.RELATIVE)
#uses VActual Register to  move the motor
# finishedsuccessfully = tmc.set_vactual_rpm(30, revolutions=10)


if finishedsuccessfully is True:
    print("Movement finished successfully")
else:
    print("Movement was not completed")





#-----------------------------------------------------------------------
# homing
# 1. param: DIAG pin
# 2. param: maximum number of revolutions. Can be negative for inverse direction
# 3. param(optional): StallGuard detection threshold
#-----------------------------------------------------------------------
#tmc.do_homing(26, 1, 50)





#-----------------------------------------------------------------------
# deactivate the motor current output
#-----------------------------------------------------------------------
tmc.set_motor_enabled(False)

print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
