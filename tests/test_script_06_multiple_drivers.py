from src.TMC_2209.TMC_2209_StepperDriver import *
import time


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_en, pin_step, pin_dir here
#-----------------------------------------------------------------------
tmc1 = TMC_2209(21, 16, 20, driver_address=0)
tmc2 = TMC_2209(26, 13, 19, driver_address=1)





#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc1.setLoglevel(Loglevel.debug)
tmc1.setMovementAbsRel(MovementAbsRel.absolute)

tmc2.setLoglevel(Loglevel.debug)
tmc2.setMovementAbsRel(MovementAbsRel.absolute)



#-----------------------------------------------------------------------
# these functions read and print the current settings in the TMC register
#-----------------------------------------------------------------------

print("---")
print("IOIN tmc1")
print("---")
tmc1.readIOIN()

print("---\n---")


print("---")
print("IOIN tmc2")
print("---")
tmc2.readIOIN()

print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
tmc1.setMotorEnabled(False)
tmc2.setMotorEnabled(False)
del tmc1

print("---")
print("SCRIPT FINISHED")
print("---")
