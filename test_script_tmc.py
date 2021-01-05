from TMC_2209_StepperDriver import TMC_2209
import time
from run_profiles import Run_profiles

print("---")
print("SCRIPT START")
print("---")

tmc = TMC_2209()


tmc.setDirection_reg(False)
tmc.setVSense(True)
tmc.setCurrent(400)
tmc.setIScaleAnalog(True)
tmc.setInterpolation(True)
tmc.setMicrosteppingResolution(2)
tmc.setInternalRSense(False)

tmc.readIOIN()
tmc.readCHOPCONF()
tmc.readDRVSTATUS()
tmc.readGCONF()


tmc.setAcceleration(500)
tmc.setMaxSpeed(1000)

tmc.setMotorEnabled(True)

tmc.runToPositionSteps(100)  #move 100 steps (1/4 revolutions (1.8° Motor))

tmc.setMotorEnabled(False)



del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
