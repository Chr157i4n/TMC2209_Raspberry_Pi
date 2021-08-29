from TMC_2209.TMC_2209_StepperDriver import *
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
tmc.setVSense(True)
tmc.setCurrent(300)
tmc.setIScaleAnalog(True)
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
# this function test whether the connection of the DIR, STEP and EN pin
# between Raspberry Pi and TMC driver is working
#-----------------------------------------------------------------------
# tmc.testDirStepEn()

# print("---\n---")





#-----------------------------------------------------------------------
# set the Accerleration and maximal Speed
#-----------------------------------------------------------------------
tmc.setAcceleration(2000)
tmc.setMaxSpeed(500)





#-----------------------------------------------------------------------
# activate the motor current output
#-----------------------------------------------------------------------
# tmc.setMotorEnabled(True)





#-----------------------------------------------------------------------
# move the motor 1 revolution
#-----------------------------------------------------------------------
# tmc.runToPositionSteps(400)                             #move to position 400
# tmc.runToPositionSteps(0)                               #move to position 0


# tmc.runToPositionSteps(400, MovementAbsRel.relative)    #move 400 steps forward
# tmc.runToPositionSteps(-400, MovementAbsRel.relative)   #move 400 steps backward


# tmc.runToPositionSteps(400)                             #move to position 400
# tmc.runToPositionSteps(0)                               #move to position 0





#-----------------------------------------------------------------------
# set a callback function for the stallguard interrupt based detection
# 1. param: pin connected to the tmc DIAG output
# 2. param: is the threshold StallGuard
# 3. param: is the callback function (threaded)
# 4. param (optional): min speed threshold (in steptime measured  in  clock  cycles)
#-----------------------------------------------------------------------
# def my_callback(channel):  
#     print("StallGuard!")
#     tmc.stop()

# tmc.setStallguard_Callback(26, 50, my_callback) # after this function call, StallGuard is active

# tmc.runToPositionSteps(4000, MovementAbsRel.relative)    #move 4000 steps forward
# time.sleep(1)
# tmc.runToPositionSteps(-400, MovementAbsRel.relative)    #move 400 steps backwards





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
