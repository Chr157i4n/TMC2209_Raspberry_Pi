from TMC_2209.TMC_2209_StepperDriver import *
import time

print("---")
print("SCRIPT START")
print("---")




tmc = TMC_2209(16, 20, 21) #use your pins for pin_step, pin_dir, pin_en here

tmc.setLoglevel(Loglevel.debug)
tmc.setMovementAbsRel(MovementAbsRel.absolute)

tmc.setDirection_reg(False)
tmc.setVSense(True)
tmc.setCurrent(400)
tmc.setIScaleAnalog(True)
tmc.setInterpolation(True)
tmc.setSpreadCycle(False)
tmc.setMicrosteppingResolution(2)
tmc.setInternalRSense(False)

tmc.readIOIN()
tmc.readCHOPCONF()
tmc.readDRVSTATUS()
tmc.readGCONF()

# print("---\n---")
# tmc.testDirStepEn()

# tmc.setAcceleration(2000)
# tmc.setMaxSpeed(1500)

# tmc.setMotorEnabled(True)

# tmc.runToPositionSteps(400)                             #move to position 400
# tmc.runToPositionSteps(0)                               #move to position 0

# tmc.runToPositionSteps(400, MovementAbsRel.relative)    #move 400 steps forward
# tmc.runToPositionSteps(-400, MovementAbsRel.relative)   #move 400 steps backward

# tmc.runToPositionSteps(400)                             #move to position 400
# tmc.runToPositionSteps(0)                               #move to position 0

# tmc.setMotorEnabled(False)





del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
