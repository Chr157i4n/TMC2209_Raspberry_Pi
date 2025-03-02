#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
test file for testing movement of motors with threads
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
    tmc1 = Tmc2209(TmcMotionControlStepDir(16, 20), 21, TmcComUart("/dev/serial0"), loglevel=Loglevel.DEBUG)
elif BOARD == Board.RASPBERRY_PI5:
    tmc1 = Tmc2209(TmcMotionControlStepDir(16, 20), 21, TmcComUart("/dev/ttyAMA0"), loglevel=Loglevel.DEBUG)
elif BOARD == Board.NVIDIA_JETSON:
    tmc1 = Tmc2209(TmcMotionControlStepDir(6, 5), 13, TmcComUart("/dev/ttyTHS1"), loglevel=Loglevel.DEBUG)
else:
    # just in case
    tmc1 = Tmc2209(TmcMotionControlStepDir(16, 20), 21, TmcComUart("/dev/serial0"), loglevel=Loglevel.DEBUG)

tmc_driverlist = [tmc1]




#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc1.tmc_logger.loglevel = Loglevel.DEBUG
tmc1.movement_abs_rel = MovementAbsRel.ABSOLUTE




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

    tmc.acceleration_fullstep = 1000
    tmc.max_speed_fullstep = 250


print("---\n---")


#-----------------------------------------------------------------------
# run part
#-----------------------------------------------------------------------

# move 4000 steps forward
tmc1.tmc_mc.run_to_position_steps_threaded(4000, MovementAbsRel.RELATIVE)

time.sleep(1)
tmc1.tmc_mc.stop()     # stop the movement after 1 second

tmc1.tmc_mc.wait_for_movement_finished_threaded()

# move 4000 steps backward
tmc1.tmc_mc.run_to_position_steps_threaded(-4000, MovementAbsRel.RELATIVE)


# while the motor is still moving
while tmc1.movement_phase != MovementPhase.STANDSTILL:
    # print the current movement phase
    print(tmc1.movement_phase)
    time.sleep(0.02)

tmc1.tmc_mc.wait_for_movement_finished_threaded()





print("---\n---")


#-----------------------------------------------------------------------
# deinitiate the Tmc2209 class
#-----------------------------------------------------------------------
tmc1.set_motor_enabled(False)
del tmc1


print("---")
print("SCRIPT FINISHED")
print("---")
