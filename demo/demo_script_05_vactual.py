#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
test file for testing the VActual
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
# use your pin for pin_en here
#-----------------------------------------------------------------------
if BOARD == Board.RASPBERRY_PI:
    tmc = TMC_2209(21, 16, 20)
if BOARD == Board.NVIDIA_JETSON:
    tmc = TMC_2209(13, 6, 5, serialport="/dev/ttyTHS1")
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
# activate the motor current output
#-----------------------------------------------------------------------
tmc.set_motor_enabled(True)





#-----------------------------------------------------------------------
# move the motor for 1 second forward, stop for 1 second
# and then move backwards for 1 second
#-----------------------------------------------------------------------
#tmc.set_vactual_dur(400)
#time.sleep(1)
#tmc.set_vactual_dur(0)
#time.sleep(1)
#tmc.set_vactual_dur(-400)
#time.sleep(1)
#tmc.set_vactual_dur(0)





#-----------------------------------------------------------------------
# set_vactual_rps uses revolutions per seconds as parameter
#-----------------------------------------------------------------------
# tmc.set_vactual_rps(1)
# time.sleep(1)
# tmc.set_vactual_rps(0)
# time.sleep(1)
# tmc.set_vactual_rps(-1)
# time.sleep(1)
# tmc.set_vactual_rps(0)





#-----------------------------------------------------------------------
# set_vactual_rps uses revolutions per seconds as parameter
#-----------------------------------------------------------------------
#tmc.set_vactual_rpm(60)
#time.sleep(1)
#tmc.set_vactual(0)
#time.sleep(1)
#tmc.set_vactual_rpm(-60)
#time.sleep(1)
#tmc.set_vactual(0)





#-----------------------------------------------------------------------
# set_vactual_rpm and set_vactual_rps accept "revolutions" and "duration"
# as keyword parameter if duration is set the script will set VActual
# to that rpm for that duration and stop the motor afterwards if revolutions
# the script will calculate the duration based on the speed and the revolutions
# Movement of the Motor will not be very accurate with this way
#-----------------------------------------------------------------------
tmc.set_vactual_rpm(30, revolutions=2)
tmc.set_vactual_rpm(-120, revolutions=2)
time.sleep(1)
tmc.set_vactual_rpm(30, duration=4)
tmc.set_vactual_rpm(-120, duration=1)





#-----------------------------------------------------------------------
# use acceleration (velocity ramping) with VActual
# does not work with revolutions as parameter
#-----------------------------------------------------------------------
# tmc.set_vactual_rpm(-120, duration=10, acceleration=500)





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
