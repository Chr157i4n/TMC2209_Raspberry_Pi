from TMC_2209.TMC_2209_StepperDriver import TMC_2209
from TMC_2209.TMC_2209_StepperDriver import Loglevel
import time

print("---")
print("SCRIPT START")
print("---")




tmc = TMC_2209(16, 20, 21) #use your pins for pin_step, pin_dir, pin_en here

tmc.setLoglevel(Loglevel.info)

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

# tmc.runToPositionSteps(400)  #move 400 steps (1 revolutions (1.8° Motor))
# tmc.runToPositionSteps(0)  #move 400 steps back to the start (1 revolutions (1.8° Motor))

# tmc.setMotorEnabled(False)





del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
