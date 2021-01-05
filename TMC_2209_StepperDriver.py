from TMC_2209_uart import TMC_UART
import TMC_2209_reg as reg
import RPi.GPIO as GPIO
import time
from enum import Enum
import math
import statistics



class Direction(Enum):
    CCW = 0
    CW = 1



class TMC_2209:
    
    tmc_uart = TMC_UART()
    _pin_step = 19
    _pin_dir = 26
    _pin_en = 13

    _direction = True

    _msres = 0
    _stepsPerRevolution = 0
    

    _currentPos = 0                 # current position of stepper in steps
    _targetPos = 0                  # the target position in steps
    _speed = 0.0                    # the current speed in steps per second
    _maxSpeed = 1.0                 # the maximum speed in steps per second
    _maxSpeedHoming = 500           # the maximum speed in steps per second for homing
    _acceleration = 1.0             # the acceleration in steps per second per second
    _accelerationHoming = 10000     # the acceleration in steps per second per second for homing
    _sqrt_twoa = 1.0                # Precomputed sqrt(2*_acceleration)
    _stepInterval = 0               # the current interval between two steps
    _minPulseWidth = 1              # minimum allowed pulse with in microseconds
    _lastStepTime = 0               # The last step time in microseconds
    _n = 0                          # step counter
    _c0 = 0                         # Initial step size in microseconds
    _cn = 0                         # Last step size in microseconds
    _cmin = 0                       # Min step size in microseconds based on maxSpeed
    _sg_threshold = 50              # threshold for stallguard
    
    
    def __init__(self):
        print("TMC2209 Init")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin_step, GPIO.OUT)
        GPIO.setup(self._pin_dir, GPIO.OUT)
        GPIO.setup(self._pin_en, GPIO.OUT)
        GPIO.output(self._pin_dir, self._direction)
        print("GPIO Init finished")      
        self.getStepsPerRevolution()
        print("TMC2209 Init finished")
        self.tmc_uart.flushSerialBuffer()


    def __del__(self):
        self.setMotorEnabled(0)
        GPIO.cleanup() 



#-----------------------------------------------------------------------
# read the register Adress "DRVSTATUS" and prints all current setting
#-----------------------------------------------------------------------
    def readDRVSTATUS(self):
        print("---")
        print("DRIVER STATUS:")
        drvstatus =self.tmc_uart.read_int(reg.DRVSTATUS)
        print(bin(drvstatus))
        if(drvstatus & reg.stst):
            print("Info: motor is standing still")
        else:
            print("Info: motor is running")

        if(drvstatus & reg.stealth):
            print("Info: motor is running on StealthChop")
        else:
            print("Info: motor is running on SpreadCycle")

        cs_actual = drvstatus & reg.cs_actual
        cs_actual = cs_actual >> 16
        print("CS actual: "+str(cs_actual))

        if(drvstatus & reg.olb):
            print("Warning: Open load detected on phase B")
        
        if(drvstatus & reg.ola):
            print("Warning: Open load detected on phase A")
        
        if(drvstatus & reg.s2vsb):
            print("Error: Short on low-side MOSFET detected on phase B. The driver becomes disabled")

        if(drvstatus & reg.s2vsa):
            print("Error: Short on low-side MOSFET detected on phase A. The driver becomes disabled")

        if(drvstatus & reg.s2gb):
            print("Error: Short to GND detected on phase B. The driver becomes disabled. ")
        
        if(drvstatus & reg.s2ga):
            print("Error: Short to GND detected on phase A. The driver becomes disabled. ")
        
        if(drvstatus & reg.ot):
            print("Error: Driver Overheating!")
        
        if(drvstatus & reg.otpw):
            print("Warning: Driver Overheating Prewarning!")
        
        print("---")
            

#-----------------------------------------------------------------------
# read the register Adress "GCONF" and prints all current setting
#-----------------------------------------------------------------------
    def readGCONF(self):
        print("---")
        print("GENERAL CONFIG")
        gconf = self.tmc_uart.read_int(reg.GCONF)
        print(bin(gconf))

        if(gconf & reg.i_scale_analog):
            print("Driver is using voltage supplied to VREF as current reference")
        else:
            print("Driver is using internal reference derived from 5VOUT")
        if(gconf & reg.internal_rsense):
            print("Internal sense resistors. Use current supplied into VREF as reference.")
            print("VREF pin internally is driven to GND in this mode.")
        else:
            print("Operation with external sense resistors")
        if(gconf & reg.en_spreadcycle):
            print("SpreadCycle mode enabled")
        else:
            print("StealthChop PWM mode enabled")
        if(gconf & reg.shaft):
            print("Inverse motor direction")
        else:
            print("normal motor direction")
        if(gconf & reg.index_otpw):
            print("INDEX pin outputs overtemperature prewarning flag")
        else:
            print("INDEX shows the first microstep position of sequencer")
        if(gconf & reg.index_step):
            print("INDEX output shows step pulses from internal pulse generator")
        else:
            print("INDEX output as selected by index_otpw")
        if(gconf & reg.mstep_reg_select):
            print("Microstep resolution selected by MSTEP register")
        else:
            print("Microstep resolution selected by pins MS1, MS2")
        
        print("---")


#-----------------------------------------------------------------------
# read the register Adress "IOIN" and prints all current setting
#-----------------------------------------------------------------------
    def readIOIN(self):
        print("---")
        print("INPUTS")
        ioin = self.tmc_uart.read_int(reg.IOIN)
        print(bin(ioin))
        if(ioin & 1<<reg.io_spread):
            print("spread is high")
        else:
            print("spread is low")

        if(ioin & 1<<reg.io_dir):
            print("dir is high")
        else:
            print("dir is low")
        
        print("---")


#-----------------------------------------------------------------------
# read the register Adress "CHOPCONF" and prints all current setting
#-----------------------------------------------------------------------
    def readCHOPCONF(self):
        print("---")
        print("CHOPPER CONTROL")
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        print(bin(chopconf))
        
        print("native "+str(self.getMicroSteppingResolution())+" microstep setting")
        
        if(chopconf & reg.intpol):
            print("interpolation to 256 microsteps")
        
        if(chopconf & reg.vsense):
            print("1: High sensitivity, low sense resistor voltage")
        else:
            print("0: Low sensitivity, high sense resistor voltage")

        print("---")





#-----------------------------------------------------------------------
# enables or disables the motor current output
#-----------------------------------------------------------------------
    def setMotorEnabled(self, en):
        GPIO.output(self._pin_en, not en)
        print("Motor output active: {}".format(en))
      

#-----------------------------------------------------------------------
# homes the motor in the given direction using stallguard
#-----------------------------------------------------------------------
    def doHoming(self, direction):
        print("---")
        print("homing")
        sg_results = []
        
        self.setDirection_pin(direction)
        self.setSpreadCycle(0)

        if (direction == 1):
            self._targetPos = self._stepsPerRevolution * 10
        else:
            self._targetPos = -self._stepsPerRevolution * 10
        self._stepInterval = 0
        self._speed = 0.0
        self._n = 0
        self.setAcceleration(10000)
        self.setMaxSpeed(self._maxSpeedHoming)
        self.computeNewSpeed()


        step_counter=0
        #print("Steps per Revolution: "+str(self._stepsPerRevolution))
        while (step_counter<self._stepsPerRevolution):
            if (self.runSpeed()): #returns true, when a step is made
                step_counter += 1
                self.computeNewSpeed()
                sg_result = self.getStallguard_Result()
                sg_results.append(sg_result)
                if(len(sg_results)>20):
                    sg_result_average = statistics.mean(sg_results[-6:])
                    if(sg_result_average < self._sg_threshold):
                        break

        if(step_counter<self._stepsPerRevolution):
            print("homing successful")
            print("Stepcounter: "+str(step_counter))
            self._currentPos = 0
        else:
            print("homing failed")
            print("Stepcounter: "+str(step_counter))
            print(sg_results)
        
        
        print("---")
        

#-----------------------------------------------------------------------
# returns the current motor position in microsteps
#-----------------------------------------------------------------------
    def getCurrentPosition(self):
        return self._currentPos


#-----------------------------------------------------------------------
# overwrites the current motor position in microsteps
#-----------------------------------------------------------------------
    def setCurrentPosition(self, newPos):
        self._currentPos = newPos

       
#-----------------------------------------------------------------------
# reverses the motor shaft direction
#-----------------------------------------------------------------------
    def reverseDirection_pin(self):
        self._direction = not self._direction
        GPIO.output(self._pin_dir, self.direction)


#-----------------------------------------------------------------------
# sets the motor shaft direction to the given value: 0 = CCW; 1 = CW
#-----------------------------------------------------------------------
    def setDirection_pin(self, direction):
        self._direction = direction
        GPIO.output(self._pin_dir, direction)


#-----------------------------------------------------------------------
# returns the motor shaft direction: 0 = CCW; 1 = CW
#-----------------------------------------------------------------------
    def getDirection_reg(self):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        return (gconf & reg.shaft)


#-----------------------------------------------------------------------
# sets the motor shaft direction to the given value: 0 = CCW; 1 = CW
#-----------------------------------------------------------------------
    def setDirection_reg(self, direction):
        ifcnt1 = self.getInterfaceTransmissionCounter()
        
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(direction):
            print("write inverse motor direction")
            gconf = self.tmc_uart.set_bit(gconf, reg.shaft)
        else:
            print("write normal motor direction")
            gconf = self.tmc_uart.clear_bit(gconf, reg.shaft)
        self.tmc_uart.write_reg(reg.GCONF, gconf)

        ifcnt2 = self.getInterfaceTransmissionCounter()
        if(ifcnt1+1 != ifcnt2):
            print("writing not successful")
            return False
        return True
        
#-----------------------------------------------------------------------
# return whether Vref (1) or 5V (0) is used for current scale
#-----------------------------------------------------------------------
    def getIScaleAnalog(self):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        return (gconf & reg.i_scale_analog)


#-----------------------------------------------------------------------
# sets Vref (1) or 5V (0) for current scale
#-----------------------------------------------------------------------
    def setIScaleAnalog(self,en):
        ifcnt1 = self.getInterfaceTransmissionCounter()
        
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(en):
            print("activated Vref for current scale")
            gconf = self.tmc_uart.set_bit(gconf, reg.i_scale_analog)
        else:
            print("activated 5V-out for current scale")
            gconf = self.tmc_uart.clear_bit(gconf, reg.i_scale_analog)
        self.tmc_uart.write_reg(reg.GCONF, gconf)

        ifcnt2 = self.getInterfaceTransmissionCounter()
        if(ifcnt1+1 != ifcnt2):
            print("writing not successful")
            return False
        return True



#-----------------------------------------------------------------------
# returns which sense resistor voltage is used for current scaling
# 0: Low sensitivity, high sense resistor voltage
# 1: High sensitivity, low sense resistor voltage
#-----------------------------------------------------------------------
    def getVSense(self):
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        return (chopconf & reg.vsense)


#-----------------------------------------------------------------------
# sets which sense resistor voltage is used for current scaling
# 0: Low sensitivity, high sense resistor voltage
# 1: High sensitivity, low sense resistor voltage
#-----------------------------------------------------------------------
    def setVSense(self,en):
        ifcnt1 = self.getInterfaceTransmissionCounter()
        
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        if(en):
            print("activated High sensitivity, low sense resistor voltage")
            chopconf = self.tmc_uart.set_bit(chopconf, reg.vsense)
        else:
            print("activated Low sensitivity, high sense resistor voltage")
            chopconf = self.tmc_uart.clear_bit(chopconf, reg.vsense)
        self.tmc_uart.write_reg(reg.CHOPCONF, chopconf)

        ifcnt2 = self.getInterfaceTransmissionCounter()
        if(ifcnt1+1 != ifcnt2):
            print("writing not successful")
            return False
        return True


#-----------------------------------------------------------------------
# returns which sense resistor voltage is used for current scaling
# 0: Low sensitivity, high sense resistor voltage
# 1: High sensitivity, low sense resistor voltage
#-----------------------------------------------------------------------
    def getInternalRSense(self):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        return (gconf & reg.internal_rsense)


#-----------------------------------------------------------------------
# sets which sense resistor voltage is used for current scaling
# 0: Low sensitivity, high sense resistor voltage
# 1: High sensitivity, low sense resistor voltage
#-----------------------------------------------------------------------
    def setInternalRSense(self,en):
        ifcnt1 = self.getInterfaceTransmissionCounter()
        
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(en):
            print("activated internal sense resistors.")
            gconf = self.tmc_uart.set_bit(gconf, reg.internal_rsense)
        else:
            print("activated operation with external sense resistors")
            gconf = self.tmc_uart.clear_bit(gconf, reg.internal_rsense)
        self.tmc_uart.write_reg(reg.GCONF, gconf)

        ifcnt2 = self.getInterfaceTransmissionCounter()
        if(ifcnt1+1 != ifcnt2):
            print("writing not successful")
            return False
        return True
    
#-----------------------------------------------------------------------
# sets the current scale (CS) for Running and Holding
# and the delay, when to be switched to Holding current
# IHold = 0-31; IRun = 0-31; IHoldDelay = 0-15
#-----------------------------------------------------------------------
    def setIRun_Ihold(self, IHold, IRun, IHoldDelay):
        ifcnt1 = self.getInterfaceTransmissionCounter()

        ihold_irun = 0
        
        ihold_irun = ihold_irun | IHold << 0
        ihold_irun = ihold_irun | IRun << 8
        ihold_irun = ihold_irun | IHoldDelay << 16
        print("ihold_irun")
        print(bin(ihold_irun))

        print("writing ihold_irun")
        self.tmc_uart.write_reg(reg.IHOLD_IRUN, ihold_irun)

        ifcnt2 = self.getInterfaceTransmissionCounter()
        if(ifcnt1+1 != ifcnt2):
            print("writing not successful")
            return False
        return True


#-----------------------------------------------------------------------
# sets the current flow for the motor
# run_current in mA
#-----------------------------------------------------------------------
    def setCurrent(self, run_current, hold_current_multiplier = 0.5, hold_current_delay = 10, Vref = 0.8):
        CS_IRun = 0
        Rsense = 0.11
        Vfs = 0

        if(self.getVSense()):
            print("Vsense: 1")
            Vfs = 0.180 * Vref / 2.5
        else:
            print("Vsense: 0")
            Vfs = 0.325 * Vref / 2.5
            
        CS_IRun = 32.0*1.41421*run_current/1000.0*(Rsense+0.02)/Vfs - 1

        CS_IRun = min(CS_IRun, 31)
        CS_IRun = max(CS_IRun, 0)
        
        CS_IHold = hold_current_multiplier * CS_IRun

        CS_IRun = round(CS_IRun)
        CS_IHold = round(CS_IHold)
        hold_current_delay = round(hold_current_delay)

        print("CS_IRun: " + str(CS_IRun))
        print("CS_IHold: " + str(CS_IHold))
        print("Delay: " + str(hold_current_delay))

        self.setIRun_Ihold(CS_IHold, CS_IRun, hold_current_delay)

       
#-----------------------------------------------------------------------
# return whether spreadcycle (1) is active or stealthchop (0)
#-----------------------------------------------------------------------
    def getSpreadCycle(self):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        return (gconf & reg.en_spreadcycle)


#-----------------------------------------------------------------------
# enables spreadcycle (1) or stealthchop (0)
#-----------------------------------------------------------------------
    def setSpreadCycle(self,en_spread):
        ifcnt1 = self.getInterfaceTransmissionCounter()
        
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(en_spread):
            print("activated Spreadcycle")
            gconf = self.tmc_uart.set_bit(gconf, reg.en_spreadcycle)
        else:
            print("activated Stealthchop")
            gconf = self.tmc_uart.clear_bit(gconf, reg.en_spreadcycle)
        self.tmc_uart.write_reg(reg.GCONF, gconf)

        ifcnt2 = self.getInterfaceTransmissionCounter()
        if(ifcnt1+1 != ifcnt2):
            print("writing not successful")
            return False
        return True


#-----------------------------------------------------------------------
# enables the tmc inbuilt interpolation of the steps to 256 microsteps
#-----------------------------------------------------------------------
    def setInterpolation(self, en):
        ifcnt1 = self.getInterfaceTransmissionCounter()
        
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)

        if(en):
            chopconf = self.tmc_uart.set_bit(chopconf, reg.intpol)
        else:
            chopconf = self.tmc_uart.clear_bit(chopconf, reg.intpol)

        print("writing microstep interpolation setting: "+str(en))
        self.tmc_uart.write_reg(reg.CHOPCONF, chopconf)
        
        ifcnt2 = self.getInterfaceTransmissionCounter()
        if(ifcnt1+1 != ifcnt2):
            print("writing not successful")
            return False
        return True


#-----------------------------------------------------------------------
# return whether the tmc inbuilt interpolation is active
#-----------------------------------------------------------------------
    def getInterpolation(self):
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        if(chopconf & reg.intpol):
            return True
        else:
            return False
    
#-----------------------------------------------------------------------
# returns the current native microstep resolution (1-256)
#-----------------------------------------------------------------------
    def getMicroSteppingResolution(self):
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        
        msresdezimal = chopconf & (reg.msres0 | reg.msres1 | reg.msres2 | reg.msres3)
        
        msresdezimal = msresdezimal >> 24
        msresdezimal = 8 - msresdezimal
                
        self.msres = int(math.pow(2, msresdezimal))
        
        return self.msres


#-----------------------------------------------------------------------
# sets the current native microstep resolution (1,2,4,8,16,32,64,128,256)
#-----------------------------------------------------------------------
    def setMicrosteppingResolution(self, msres):
        ifcnt1 = self.getInterfaceTransmissionCounter()
        
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        chopconf = chopconf & (~reg.msres0 | ~reg.msres1 | ~reg.msres2 | ~reg.msres3) #setting all bits to zero
        msresdezimal = int(math.log(msres, 2))
        msresdezimal = 8 - msresdezimal
        chopconf = int(chopconf) & int(4043309055)
        chopconf = chopconf | msresdezimal <<24
        
        print("writing "+str(msres)+" microstep setting")
        self.tmc_uart.write_reg(reg.CHOPCONF, chopconf)
        ifcnt2 = self.getInterfaceTransmissionCounter()
        if(ifcnt1+1 != ifcnt2):
            print("writing not successful")
            print("ifcnt1: "+str(ifcnt1)+"\tifcnt2: "+str(ifcnt2))
            #return False
        
        self.setMStepResolutionRegSelect(True)

        return True
        

#-----------------------------------------------------------------------
# sets the register bit "mstep_reg_select" to 1 or 0 depending to the given value.
# this is needed to set the microstep resolution via UART
# this method is called by "setMicrosteppingResolution"
#-----------------------------------------------------------------------
    def setMStepResolutionRegSelect(self, en):                  
        ifcnt1 = self.getInterfaceTransmissionCounter()
        
        gconf = self.tmc_uart.read_int(reg.GCONF)
        
        if(en == True):
            gconf = self.tmc_uart.set_bit(gconf, reg.mstep_reg_select)
        else:
            gconf = self.tmc_uart.clear_bit(gconf, reg.mstep_reg_select)

        print("writing MStep Reg Select: "+str(en))
        self.tmc_uart.write_reg(reg.GCONF, gconf)
        
        ifcnt2 = self.getInterfaceTransmissionCounter()
        if(ifcnt1+1 != ifcnt2):
            print("writing not successful")
            print("ifcnt1: "+str(ifcnt1)+"\tifcnt2: "+str(ifcnt2))
            return False
        return True
        

#-----------------------------------------------------------------------
# returns how many steps are needed for one revolution
#-----------------------------------------------------------------------
    def getStepsPerRevolution(self):
        self._stepsPerRevolution = 200*self.getMicroSteppingResolution()
        return self._stepsPerRevolution


#-----------------------------------------------------------------------
# reads the interface transmission counter from the tmc register
# this value is increased on every succesfull write access
# can be used to verify a write access
#-----------------------------------------------------------------------
    def getInterfaceTransmissionCounter(self):
        ifcnt = self.tmc_uart.read_int(reg.IFCNT)
        print("Interface Transmission Counter: "+str(ifcnt))
        return ifcnt
        


#-----------------------------------------------------------------------
# return the current stallguard result
# its will be calculated with every fullstep
# higher values means a lower motor load
#-----------------------------------------------------------------------
    def getStallguard_Result(self):
        sg_result = self.tmc_uart.read_int(reg.SG_RESULT)
        return sg_result


#-----------------------------------------------------------------------
# sets the maximum motor speed in steps per second
#-----------------------------------------------------------------------
    def setMaxSpeed(self, speed):
        if (speed < 0.0):
           speed = -speed
        if (self._maxSpeed != speed):
            self._maxSpeed = speed
            self._cmin = 1000000.0 / speed
            # Recompute _n from current speed and adjust speed if accelerating or cruising
            if (self._n > 0):
                self._n = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
                self.computeNewSpeed()


#-----------------------------------------------------------------------
# returns the maximum motor speed in steps per second
#-----------------------------------------------------------------------
    def getMaxSpeed(self):
        return self._maxSpeed


#-----------------------------------------------------------------------
# sets the motor acceleration/decceleration in steps per sec per sec
#-----------------------------------------------------------------------
    def setAcceleration(self, acceleration):
        if (acceleration == 0.0):
            return
        if (acceleration < 0.0):
          acceleration = -acceleration
        if (self._acceleration != acceleration):
            # Recompute _n per Equation 17
            self._n = self._n * (self._acceleration / acceleration)
            # New c0 per Equation 7, with correction per Equation 15
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
            self._acceleration = acceleration
            self.computeNewSpeed()


#-----------------------------------------------------------------------
# returns the motor acceleration/decceleration in steps per sec per sec
#-----------------------------------------------------------------------
    def getAcceleration(self):
        return self._acceleration


#-----------------------------------------------------------------------
# runs the motor to the given position.
# with acceleration and deceleration
# blocks the code until finished!
#-----------------------------------------------------------------------
    def runToPositionSteps(self, steps):
        self._targetPos = steps
        self._stepInterval = 0
        self._speed = 0.0
        self._n = 0
        self.computeNewSpeed()
        while (self.run()): #returns false, when target position is reached
            pass


#-----------------------------------------------------------------------
# runs the motor to the given position.
# with acceleration and deceleration
# blocks the code until finished!
#-----------------------------------------------------------------------
    def runToPositionRevolutions(self, revolutions):
        self.runToPositionSteps(round(revolutions * self._stepsPerRevolution))


#-----------------------------------------------------------------------
# calculates a new speed if a speed was made
# returns true if the target position is reached
# should not be called from outside!
#-----------------------------------------------------------------------
    def run(self):
        if (self.runSpeed()): #returns true, when a step is made
            self.computeNewSpeed()
            #print("distance to go: " + str(self.distanceToGo()) + "speed: " + str(self._speed))
        return (self._speed != 0.0 and self.distanceToGo() != 0)


#-----------------------------------------------------------------------
# returns the remaining distance the motor should run
#-----------------------------------------------------------------------
    def distanceToGo(self):
        return self._targetPos - self._currentPos


#-----------------------------------------------------------------------
# returns the calculated current speed depending on the acceleration
#-----------------------------------------------------------------------
    def computeNewSpeed(self):
        distanceTo = self.distanceToGo() # +ve is clockwise from curent location
        #print("distanceTo: " + str(distanceTo))
        stepsToStop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
        if (distanceTo == 0 and stepsToStop <= 1):
            # We are at the target and its time to stop
            self._stepInterval = 0
            self._speed = 0.0
            self._n = 0
            #print("time to stop")
            return
        
        if (distanceTo > 0):
            # We are anticlockwise from the target
            # Need to go clockwise from here, maybe decelerate now
            if (self._n > 0):
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= distanceTo) or self._direction == Direction.CCW):
                    self._n = -stepsToStop # Start deceleration
            elif (self._n < 0):
                # Currently decelerating, need to accel again?
                if ((stepsToStop < distanceTo) and self._direction == Direction.CW):
                    self._n = -self._n # Start accceleration
        elif (distanceTo < 0):
            # We are clockwise from the target
            # Need to go anticlockwise from here, maybe decelerate
            if (self._n > 0):
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= -distanceTo) or self._direction == Direction.CW):
                    self._n = -stepsToStop # Start deceleration
            elif (self._n < 0):
                # Currently decelerating, need to accel again?
                if ((stepsToStop < -distanceTo) and self._direction == Direction.CCW):
                    self._n = -self._n # Start accceleration
        # Need to accelerate or decelerate
        if (self._n == 0):
            # First step from stopped
            self._cn = self._c0
            #direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW
            #print("distance to: " + str(distanceTo))
            if(distanceTo > 0):
                self.setDirection_pin(1)
                #print("going CW")
            else:
                self.setDirection_pin(0)
                #print("going CCW")
        else:
            # Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1)) # Equation 13
            self._cn = max(self._cn, self._cmin)
        self._n += 1
        self._stepInterval = self._cn
        self._speed = 1000000.0 / self._cn
        if (self._direction == 0):
            self._speed = -self._speed


#-----------------------------------------------------------------------
# this methods does the actual steps with the current speed
#-----------------------------------------------------------------------
    def runSpeed(self):  
        # Dont do anything unless we actually have a step interval
        if (not self._stepInterval):
            return False
        
        
        #curtime = micros();
        curtime = time.time() * 1000 * 1000
        
        #print("current time: " + str(curtime))
        #print("last st time: " + str(self._lastStepTime))
        
        if (curtime - self._lastStepTime >= self._stepInterval):
            #print(type(self.direction))
            #print(self.direction)
            if (self._direction == 1): # Clockwise
                self._currentPos += 1
                #print("going CW")
            else: # Anticlockwise 
                self._currentPos -= 1
                #print("going CCW")
            GPIO.output(self._pin_step, GPIO.HIGH)
            self._lastStepTime = curtime # Caution: does not account for costs in step()
            return True
        elif (curtime - self._lastStepTime >= 0.5 * self._stepInterval):
            GPIO.output(self._pin_step, GPIO.LOW)
            return False
        else:
            return False
