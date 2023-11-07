#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
test file for testing movement of motors with threads
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
tmc1 = TMC_2209(21, 16, 20, driver_address=0)

tmc_driverlist = [tmc1]




#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc1.set_loglevel(Loglevel.DEBUG)
tmc1.set_movement_abs_rel(MovementAbsRel.ABSOLUTE)




#-----------------------------------------------------------------------
# these functions change settings in the TMC register
#-----------------------------------------------------------------------
for tmc in tmc_driverlist:
    tmc.set_direction_reg(False)
    tmc.set_current(300)
    tmc.set_interpolation(True)
    tmc.set_spreadcycle(False)
    tmc.set_microstepping_resolution(2)
    tmc.set_internal_rsense(False)
    tmc.set_motor_enabled(True)

    tmc.set_acceleration_fullstep(1000)
    tmc.set_max_speed_fullstep(250)


print("---\n---")


#-----------------------------------------------------------------------
# run part
#-----------------------------------------------------------------------

# move 4000 steps forward
tmc1.run_to_position_steps_threaded(4000, MovementAbsRel.RELATIVE)

time.sleep(1)
tmc1.stop()     # stop the movement after 1 second

tmc1.wait_for_movement_finished_threaded()

# move 4000 steps backward
tmc1.run_to_position_steps_threaded(-4000, MovementAbsRel.RELATIVE)


# while the motor is still moving
while tmc1.get_movement_phase() != MovementPhase.STANDSTILL:
    # print the current movement phase
    print(tmc1.get_movement_phase())
    time.sleep(0.02)

tmc1.wait_for_movement_finished_threaded()





print("---\n---")


#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
tmc1.set_motor_enabled(False)
tmc1.deinit()
del tmc1


print("---")
print("SCRIPT FINISHED")
print("---")
