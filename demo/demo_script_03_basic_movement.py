#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
test file for testing basic movement
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
if BOARD == Board.RASPBERRY_PI:
    tmc = TMC_2209(21, 16, 20, loglevel=Loglevel.DEBUG)
elif BOARD == Board.RASPBERRY_PI5:
    tmc = TMC_2209(21, 16, 20, serialport="/dev/ttyAMA0", loglevel=Loglevel.DEBUG)
elif BOARD == Board.NVIDIA_JETSON:
    tmc = TMC_2209(13, 6, 5, serialport="/dev/ttyTHS1", loglevel=Loglevel.DEBUG)
else:
    # just in case
    tmc = TMC_2209(21, 16, 20, loglevel=Loglevel.DEBUG)






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
# set the Acceleration and maximal Speed
#-----------------------------------------------------------------------
# tmc.set_acceleration(2000)
# tmc.set_max_speed(500)

#-----------------------------------------------------------------------
# set the Acceleration and maximal Speed in fullsteps
#-----------------------------------------------------------------------
tmc.set_acceleration_fullstep(1000)
tmc.set_max_speed_fullstep(250)







#-----------------------------------------------------------------------
# activate the motor current output
#-----------------------------------------------------------------------
tmc.set_motor_enabled(True)





#-----------------------------------------------------------------------
# move the motor 1 revolution
#-----------------------------------------------------------------------
tmc.run_to_position_steps(400)                             #move to position 400
tmc.run_to_position_steps(0)                               #move to position 0


tmc.run_to_position_steps(400, MovementAbsRel.RELATIVE)    #move 400 steps forward
tmc.run_to_position_steps(-400, MovementAbsRel.RELATIVE)   #move 400 steps backward


tmc.run_to_position_steps(400)                             #move to position 400
tmc.run_to_position_steps(0)                               #move to position 0





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
