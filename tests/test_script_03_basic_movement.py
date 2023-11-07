"""
test file for testing basic movement
"""

from src.TMC_2209.TMC_2209_StepperDriver import *
import time


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_en, pin_step, pin_dir here
#-----------------------------------------------------------------------
tmc = TMC_2209(21, 16, 20, loglevel=Loglevel.DEBUG)





#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.set_loglevel(Loglevel.DEBUG)
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
tmc.readIOIN()
tmc.readCHOPCONF()
tmc.readDRVSTATUS()
tmc.readGCONF()

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
tmc.deinit()
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
