#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
test file for testing basic movement
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
    tmc = Tmc2209(pin_step=16, loglevel=Loglevel.DEBUG)
elif BOARD == Board.RASPBERRY_PI5:
    tmc = Tmc2209(pin_step=16, tmc_com=TmcUart("/dev/ttyAMA0"), loglevel=Loglevel.DEBUG)
elif BOARD == Board.NVIDIA_JETSON:
    tmc = Tmc2209(pin_step=13, tmc_com=TmcUart("/dev/ttyTHS1"), loglevel=Loglevel.DEBUG)
else:
    # just in case
    tmc = Tmc2209(pin_step=21, loglevel=Loglevel.DEBUG)






#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.tmc_logger.loglevel = Loglevel.DEBUG
tmc.movement_abs_rel = MovementAbsRel.ABSOLUTE





#-----------------------------------------------------------------------
# these functions change settings in the TMC register
#-----------------------------------------------------------------------
tmc.set_toff(0)
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
tmc.acceleration_fullstep = 1000
tmc.max_speed_fullstep = 250







#-----------------------------------------------------------------------
# activate the motor current output
#-----------------------------------------------------------------------
#tmc.set_motor_enabled(True)
tmc.set_toff(3)




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
#tmc.set_motor_enabled(False)
tmc.set_toff(0)

print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the Tmc2209 class
#-----------------------------------------------------------------------
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
