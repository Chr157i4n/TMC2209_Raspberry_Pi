from src.TMC_2209.TMC_2209_StepperDriver import *
import time


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_step, pin_dir, pin_en here
#-----------------------------------------------------------------------
tmc = TMC_2209(16, 20, 21)





#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.setLoglevel(Loglevel.debug)
tmc.setMovementAbsRel(MovementAbsRel.absolute)





#-----------------------------------------------------------------------
# these functions change settings in the TMC register
#-----------------------------------------------------------------------
tmc.setDirection_reg(False)
tmc.setCurrent(300)
tmc.setInterpolation(True)
tmc.setSpreadCycle(False)
tmc.setMicrosteppingResolution(2)
tmc.setInternalRSense(False)


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
# activate the motor current output
#-----------------------------------------------------------------------
tmc.setMotorEnabled(True)





#-----------------------------------------------------------------------
# move the motor for 1 second forward, stop for 1 second
# and then move backwards for 1 second
#-----------------------------------------------------------------------
#tmc.setVActual(400)
#time.sleep(1)
#tmc.setVActual(0)
#time.sleep(1)
#tmc.setVActual(-400)
#time.sleep(1)
#tmc.setVActual(0)





#-----------------------------------------------------------------------
# setVActual_rps uses revolutions per seconds as parameter
#-----------------------------------------------------------------------
# tmc.setVActual_rps(1)
# time.sleep(1)
# tmc.setVActual_rps(0)
# time.sleep(1)
# tmc.setVActual_rps(-1)
# time.sleep(1)
# tmc.setVActual_rps(0)





#-----------------------------------------------------------------------
# setVActual_rps uses revolutions per seconds as parameter
#-----------------------------------------------------------------------
#tmc.setVActual_rpm(60)
#time.sleep(1)
#tmc.setVActual(0)
#time.sleep(1)
#tmc.setVActual_rpm(-60)
#time.sleep(1)
#tmc.setVActual(0)





#-----------------------------------------------------------------------
# setVActual_rpm and setVActual_rps accept "revolutions" and "duration" as keyword parameter
# if duration is set the script will set VActual to that rpm for that duration and stop the motor afterwards
# if revolutions the script will calculate the duration based on the speed and the revolutions
# Movement of the Motor will not be very accurate with this way
#-----------------------------------------------------------------------
tmc.setVActual_rpm(30, revolutions=2)
tmc.setVActual_rpm(-120, revolutions=2)
time.sleep(1)
tmc.setVActual_rpm(30, duration=4)
tmc.setVActual_rpm(-120, duration=1)





#-----------------------------------------------------------------------
# use acceleration (velocity ramping) with VActual
# does not work with revolutions as parameter
#-----------------------------------------------------------------------
# tmc.setVActual_rpm(-120, duration=10, acceleration=500)





#-----------------------------------------------------------------------
# deactivate the motor current output
#-----------------------------------------------------------------------
tmc.setMotorEnabled(False)

print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
