from .TMC_2209_uart import TMC_UART
from . import TMC_2209_reg as reg
import RPi.GPIO as GPIO
import time
from enum import Enum
import math
import statistics



class Direction(Enum):
    CCW = 0
    CW = 1


class Loglevel(Enum):
    none = 0
    error = 10
    info = 20
    debug = 30
    movement = 40
    all = 100

class MovementAbsRel(Enum):
    absolute = 0
    relative = 1
    





class TMC_2209:
    
    tmc_uart = None
    _pin_step = -1
    _pin_dir = -1
    _pin_en = -1

    _direction = True

    _msres = -1
    _stepsPerRevolution = 0
    
    _loglevel = Loglevel.info

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
    _sg_threshold = 100             # threshold for stallguard
    _movement_abs_rel = MovementAbsRel.absolute
    
    
    def __init__(self, pin_step, pin_dir, pin_en, baudrate=115200, serialport="/dev/serial0"):
        self.tmc_uart = TMC_UART(serialport, baudrate)
        self._pin_step = pin_step
        self._pin_dir = pin_dir
        self._pin_en = pin_en
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: Init")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin_step, GPIO.OUT)
        GPIO.setup(self._pin_dir, GPIO.OUT)
        GPIO.setup(self._pin_en, GPIO.OUT)
        GPIO.output(self._pin_dir, self._direction)
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: GPIO Init finished")      
        self.readStepsPerRevolution()
        self.clearGSTAT()
        self.tmc_uart.flushSerialBuffer()
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: Init finished")


    def __del__(self):
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: Deinit")
        self.setMotorEnabled(False)
        GPIO.cleanup() 


#-----------------------------------------------------------------------
# set the loglevel. See the Enum Loglevel
#-----------------------------------------------------------------------       
    def setLoglevel(self, loglevel):
        self._loglevel = loglevel


#-----------------------------------------------------------------------
# set whether the movment should be relative or absolute by default.
# See the Enum MovementAbsoluteRelative
#-----------------------------------------------------------------------       
    def setMovementAbsRel(self, movement_abs_rel):
        self._movement_abs_rel = movement_abs_rel


#-----------------------------------------------------------------------
# read the register Adress "DRVSTATUS" and prints all current setting
#-----------------------------------------------------------------------
    def readDRVSTATUS(self):
        print("TMC2209: ---")
        print("TMC2209: DRIVER STATUS:")
        drvstatus =self.tmc_uart.read_int(reg.DRVSTATUS)
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209:", bin(drvstatus))
        if(drvstatus & reg.stst):
            print("TMC2209: Info: motor is standing still")
        else:
            print("TMC2209: Info: motor is running")

        if(drvstatus & reg.stealth):
            print("TMC2209: Info: motor is running on StealthChop")
        else:
            print("TMC2209: Info: motor is running on SpreadCycle")

        cs_actual = drvstatus & reg.cs_actual
        cs_actual = cs_actual >> 16
        print("TMC2209: CS actual: "+str(cs_actual))

        if(drvstatus & reg.olb):
            print("TMC2209: Warning: Open load detected on phase B")
        
        if(drvstatus & reg.ola):
            print("TMC2209: Warning: Open load detected on phase A")
        
        if(drvstatus & reg.s2vsb):
            print("TMC2209: Error: Short on low-side MOSFET detected on phase B. The driver becomes disabled")

        if(drvstatus & reg.s2vsa):
            print("TMC2209: Error: Short on low-side MOSFET detected on phase A. The driver becomes disabled")

        if(drvstatus & reg.s2gb):
            print("TMC2209: Error: Short to GND detected on phase B. The driver becomes disabled. ")
        
        if(drvstatus & reg.s2ga):
            print("TMC2209: Error: Short to GND detected on phase A. The driver becomes disabled. ")
        
        if(drvstatus & reg.ot):
            print("TMC2209: Error: Driver Overheating!")
        
        if(drvstatus & reg.otpw):
            print("TMC2209: Warning: Driver Overheating Prewarning!")
        
        print("---")
        return drvstatus
            

#-----------------------------------------------------------------------
# read the register Adress "GCONF" and prints all current setting
#-----------------------------------------------------------------------
    def readGCONF(self):
        print("TMC2209: ---")
        print("TMC2209: GENERAL CONFIG")
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209:", bin(gconf))

        if(gconf & reg.i_scale_analog):
            print("TMC2209: Driver is using voltage supplied to VREF as current reference")
        else:
            print("TMC2209: Driver is using internal reference derived from 5VOUT")
        if(gconf & reg.internal_rsense):
            print("TMC2209: Internal sense resistors. Use current supplied into VREF as reference.")
            print("TMC2209: VREF pin internally is driven to GND in this mode.")
            print("TMC2209: This will most likely destroy your driver!!!")
            raise SystemExit
        else:
            print("TMC2209: Operation with external sense resistors")
        if(gconf & reg.en_spreadcycle):
            print("TMC2209: SpreadCycle mode enabled")
        else:
            print("TMC2209: StealthChop PWM mode enabled")
        if(gconf & reg.shaft):
            print("TMC2209: Inverse motor direction")
        else:
            print("TMC2209: normal motor direction")
        if(gconf & reg.index_otpw):
            print("TMC2209: INDEX pin outputs overtemperature prewarning flag")
        else:
            print("TMC2209: INDEX shows the first microstep position of sequencer")
        if(gconf & reg.index_step):
            print("TMC2209: INDEX output shows step pulses from internal pulse generator")
        else:
            print("TMC2209: INDEX output as selected by index_otpw")
        if(gconf & reg.mstep_reg_select):
            print("TMC2209: Microstep resolution selected by MSTEP register")
        else:
            print("TMC2209: Microstep resolution selected by pins MS1, MS2")
        
        print("TMC2209: ---")
        return gconf


#-----------------------------------------------------------------------
# read the register Adress "GSTAT" and prints all current setting
#-----------------------------------------------------------------------
    def readGSTAT(self):
        print("TMC2209: ---")
        print("TMC2209: GSTAT")
        gstat = self.tmc_uart.read_int(reg.GSTAT)
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209:",bin(gstat))
        if(gstat & reg.reset):
            print("TMC2209: The Driver has been reset since the last read access to GSTAT")
        if(gstat & reg.drv_err):
            print("TMC2209: The driver has been shut down due to overtemperature or short circuit detection since the last read access")
        if(gstat & reg.uv_cp):
            print("TMC2209: Undervoltage on the charge pump. The driver is disabled in this case")
        print("TMC2209: ---")
        return gstat


#-----------------------------------------------------------------------
# read the register Adress "GSTAT" and prints all current setting
#-----------------------------------------------------------------------
    def clearGSTAT(self):        
        gstat = self.tmc_uart.read_int(reg.GSTAT)
        
        gstat = self.tmc_uart.set_bit(gstat, reg.reset)
        gstat = self.tmc_uart.set_bit(gstat, reg.drv_err)
        
        self.tmc_uart.write_reg_check(reg.GSTAT, gstat)

 
#-----------------------------------------------------------------------
# read the register Adress "IOIN" and prints all current setting
#-----------------------------------------------------------------------
    def readIOIN(self):
        print("TMC2209: ---")
        print("TMC2209: INPUTS")
        ioin = self.tmc_uart.read_int(reg.IOIN)
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209:", bin(ioin))
        if(ioin & reg.io_spread):
            print("TMC2209: spread is high")
        else:
            print("TMC2209: spread is low")

        if(ioin & reg.io_dir):
            print("TMC2209: dir is high")
        else:
            print("TMC2209: dir is low")

        if(ioin & reg.io_step):
            print("TMC2209: step is high")
        else:
            print("TMC2209: step is low")

        if(ioin & reg.io_enn):
            print("TMC2209: en is high")
        else:
            print("TMC2209: en is low")
        
        print("TMC2209: ---")
        return ioin


#-----------------------------------------------------------------------
# read the register Adress "CHOPCONF" and prints all current setting
#-----------------------------------------------------------------------
    def readCHOPCONF(self):
        print("TMC2209: ---")
        print("TMC2209: CHOPPER CONTROL")
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209:", bin(chopconf))
        
        print("TMC2209: native "+str(self.getMicroSteppingResolution())+" microstep setting")
        
        if(chopconf & reg.intpol):
            print("TMC2209: interpolation to 256 microsteps")
        
        if(chopconf & reg.vsense):
            print("TMC2209: 1: High sensitivity, low sense resistor voltage")
        else:
            print("TMC2209: 0: Low sensitivity, high sense resistor voltage")

        print("TMC2209: ---")
        return chopconf





#-----------------------------------------------------------------------
# enables or disables the motor current output
#-----------------------------------------------------------------------
    def setMotorEnabled(self, en):
        GPIO.output(self._pin_en, not en)
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: Motor output active: {}".format(en))
      

#-----------------------------------------------------------------------
# homes the motor in the given direction using stallguard
#-----------------------------------------------------------------------
    def doHoming(self, direction, threshold=None):
        sg_results = []
        
        if(threshold is not None):
            self._sg_threshold = threshold
        
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: ---")
            print("TMC2209: homing")
        
        if(self._loglevel.value >= Loglevel.debug.value):
            print("TMC2209: Stallguard threshold:",self._sg_threshold)
        
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
        #print("TMC2209: Steps per Revolution: "+str(self._stepsPerRevolution))
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
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: homing successful")
                print("TMC2209: Stepcounter: "+str(step_counter))
            if(self._loglevel.value >= Loglevel.debug.value):
                print("TMC2209: Stepcounter: "+str(step_counter))
                print(sg_results)
            self._currentPos = 0
        else:
            if(self._loglevel.value >= Loglevel.error.value):
                print("TMC2209: homing failed")
            if(self._loglevel.value >= Loglevel.debug.value):
                print("TMC2209: Stepcounter: "+str(step_counter))
                print(sg_results)
        
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: ---")
        

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
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(direction):
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: write inverse motor direction")
            gconf = self.tmc_uart.set_bit(gconf, reg.shaft)
        else:
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: write normal motor direction")
            gconf = self.tmc_uart.clear_bit(gconf, reg.shaft)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)
        
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
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(en):
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: activated Vref for current scale")
            gconf = self.tmc_uart.set_bit(gconf, reg.i_scale_analog)
        else:
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: activated 5V-out for current scale")
            gconf = self.tmc_uart.clear_bit(gconf, reg.i_scale_analog)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)



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
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        if(en):
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: activated High sensitivity, low sense resistor voltage")
            chopconf = self.tmc_uart.set_bit(chopconf, reg.vsense)
        else:
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: activated Low sensitivity, high sense resistor voltage")
            chopconf = self.tmc_uart.clear_bit(chopconf, reg.vsense)
        self.tmc_uart.write_reg_check(reg.CHOPCONF, chopconf)


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
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(en):
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: activated internal sense resistors.")
            gconf = self.tmc_uart.set_bit(gconf, reg.internal_rsense)
        else:
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: activated operation with external sense resistors")
            gconf = self.tmc_uart.clear_bit(gconf, reg.internal_rsense)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)
    

#-----------------------------------------------------------------------
# sets the current scale (CS) for Running and Holding
# and the delay, when to be switched to Holding current
# IHold = 0-31; IRun = 0-31; IHoldDelay = 0-15
#-----------------------------------------------------------------------
    def setIRun_Ihold(self, IHold, IRun, IHoldDelay):
        ihold_irun = 0
        
        ihold_irun = ihold_irun | IHold << 0
        ihold_irun = ihold_irun | IRun << 8
        ihold_irun = ihold_irun | IHoldDelay << 16
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: ihold_irun")
            print(bin(ihold_irun))

            print("TMC2209: writing ihold_irun")
        self.tmc_uart.write_reg_check(reg.IHOLD_IRUN, ihold_irun)


#-----------------------------------------------------------------------
# sets the current flow for the motor
# run_current in mA
#-----------------------------------------------------------------------
    def setCurrent(self, run_current, hold_current_multiplier = 0.5, hold_current_delay = 10, Vref = 1.2):
        CS_IRun = 0
        Rsense = 0.11
        Vfs = 0

        if(self.getVSense()):
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: Vsense: 1")
            Vfs = 0.180 * Vref / 2.5
        else:
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: Vsense: 0")
            Vfs = 0.325 * Vref / 2.5
            
        CS_IRun = 32.0*1.41421*run_current/1000.0*(Rsense+0.02)/Vfs - 1

        CS_IRun = min(CS_IRun, 31)
        CS_IRun = max(CS_IRun, 0)
        
        CS_IHold = hold_current_multiplier * CS_IRun

        CS_IRun = round(CS_IRun)
        CS_IHold = round(CS_IHold)
        hold_current_delay = round(hold_current_delay)

        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: CS_IRun: " + str(CS_IRun))
            print("TMC2209: CS_IHold: " + str(CS_IHold))
            print("TMC2209: Delay: " + str(hold_current_delay))

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
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(en_spread):
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: activated Spreadcycle")
            gconf = self.tmc_uart.set_bit(gconf, reg.en_spreadcycle)
        else:
            if(self._loglevel.value >= Loglevel.info.value):
                print("TMC2209: activated Stealthchop")
            gconf = self.tmc_uart.clear_bit(gconf, reg.en_spreadcycle)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)


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
# enables the tmc inbuilt interpolation of the steps to 256 microsteps
#-----------------------------------------------------------------------
    def setInterpolation(self, en):
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)

        if(en):
            chopconf = self.tmc_uart.set_bit(chopconf, reg.intpol)
        else:
            chopconf = self.tmc_uart.clear_bit(chopconf, reg.intpol)

        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: writing microstep interpolation setting: "+str(en))
        self.tmc_uart.write_reg_check(reg.CHOPCONF, chopconf)


#-----------------------------------------------------------------------
# returns the current native microstep resolution (1-256)
#-----------------------------------------------------------------------
    def getMicroSteppingResolution(self):
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        
        msresdezimal = chopconf & (reg.msres0 | reg.msres1 | reg.msres2 | reg.msres3)
        
        msresdezimal = msresdezimal >> 24
        msresdezimal = 8 - msresdezimal
                
        self._msres = int(math.pow(2, msresdezimal))
        
        return self._msres


#-----------------------------------------------------------------------
# sets the current native microstep resolution (1,2,4,8,16,32,64,128,256)
#-----------------------------------------------------------------------
    def setMicrosteppingResolution(self, msres):      
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        chopconf = chopconf & (~reg.msres0 | ~reg.msres1 | ~reg.msres2 | ~reg.msres3) #setting all bits to zero
        msresdezimal = int(math.log(msres, 2))
        msresdezimal = 8 - msresdezimal
        chopconf = int(chopconf) & int(4043309055)
        chopconf = chopconf | msresdezimal <<24
        
        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: writing "+str(msres)+" microstep setting")
        self.tmc_uart.write_reg_check(reg.CHOPCONF, chopconf)
        
        self.setMStepResolutionRegSelect(True)

        self.readStepsPerRevolution()

        return True
        

#-----------------------------------------------------------------------
# sets the register bit "mstep_reg_select" to 1 or 0 depending to the given value.
# this is needed to set the microstep resolution via UART
# this method is called by "setMicrosteppingResolution"
#-----------------------------------------------------------------------
    def setMStepResolutionRegSelect(self, en):                  
        gconf = self.tmc_uart.read_int(reg.GCONF)
        
        if(en == True):
            gconf = self.tmc_uart.set_bit(gconf, reg.mstep_reg_select)
        else:
            gconf = self.tmc_uart.clear_bit(gconf, reg.mstep_reg_select)

        if(self._loglevel.value >= Loglevel.info.value):
            print("TMC2209: writing MStep Reg Select: "+str(en))
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)
        

#-----------------------------------------------------------------------
# returns how many steps are needed for one revolution
#-----------------------------------------------------------------------
    def readStepsPerRevolution(self):
        self._stepsPerRevolution = 200*self.getMicroSteppingResolution()
        return self._stepsPerRevolution


#-----------------------------------------------------------------------
# returns how many steps are needed for one revolution
#-----------------------------------------------------------------------
    def getStepsPerRevolution(self):
        return self._stepsPerRevolution


#-----------------------------------------------------------------------
# reads the interface transmission counter from the tmc register
# this value is increased on every succesfull write access
# can be used to verify a write access
#-----------------------------------------------------------------------
    def getInterfaceTransmissionCounter(self):
        ifcnt = self.tmc_uart.read_int(reg.IFCNT)
        if(self._loglevel.value >= Loglevel.debug.value):
            print("TMC2209: Interface Transmission Counter: "+str(ifcnt))
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
# returns the current Microstep counter.
# Indicates actual position in the microstep table for CUR_A
#-----------------------------------------------------------------------
    def getMicrostepCounter(self):
        mscnt = self.tmc_uart.read_int(reg.MSCNT)
        return mscnt


#-----------------------------------------------------------------------
# returns the current Microstep counter.
# Indicates actual position in the microstep table for CUR_A
#-----------------------------------------------------------------------
    def getMicrostepCounterInSteps(self, offset=0):
        step = (self.getMicrostepCounter()-64)*(self._msres*4)/1024
        step = (4*self._msres)-step-1
        step = round(step)
        return step+offset


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
    def runToPositionSteps(self, steps, movement_abs_rel = None):
        if(movement_abs_rel is not None):
            this_movement_abs_rel = movement_abs_rel
        else:
            this_movement_abs_rel = self._movement_abs_rel

        if(this_movement_abs_rel == MovementAbsRel.relative):
            self._targetPos = self._currentPos + steps
        else:
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
    def runToPositionRevolutions(self, revolutions, movement_absolute_relative = None):
        self.runToPositionSteps(round(revolutions * self._stepsPerRevolution), movement_absolute_relative)


#-----------------------------------------------------------------------
# calculates a new speed if a speed was made
# returns true if the target position is reached
# should not be called from outside!
#-----------------------------------------------------------------------
    def run(self):
        if (self.runSpeed()): #returns true, when a step is made
            self.computeNewSpeed()
            #print("TMC2209: distance to go: " + str(self.distanceToGo()) + "\tspeed: " + str(self._speed))
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
        #print("TMC2209: distanceTo: " + str(distanceTo))
        stepsToStop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
        if (distanceTo == 0 and stepsToStop <= 1):
            # We are at the target and its time to stop
            self._stepInterval = 0
            self._speed = 0.0
            self._n = 0
            if(self._loglevel.value >= Loglevel.movement.value):
                print("TMC2209: time to stop")
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
            GPIO.output(self._pin_step, GPIO.LOW)
            #direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW
            #print("TMC2209: distance to: " + str(distanceTo))
            if(distanceTo > 0):
                self.setDirection_pin(1)
                if(self._loglevel.value >= Loglevel.movement.value):
                    print("TMC2209: going CW")
            else:
                self.setDirection_pin(0)
                if(self._loglevel.value >= Loglevel.movement.value):
                    print("TMC2209: going CCW")
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
        #curtime = time.time() * 1000 * 1000
        curtime = time.time_ns()/1000
        
        #print("TMC2209: current time: " + str(curtime))
        #print("TMC2209: last st time: " + str(self._lastStepTime))
        
        if (curtime - self._lastStepTime >= self._stepInterval):
            #print(type(self.direction))
            #print(self.direction)
            if (self._direction == 1): # Clockwise
                self._currentPos += 1
                #print("TMC2209: going CW")
            else: # Anticlockwise 
                self._currentPos -= 1
                #print("TMC2209: going CCW")
            self.makeAStep()
            
            self._lastStepTime = curtime # Caution: does not account for costs in step()
            return True
        else:
            return False



#-----------------------------------------------------------------------
# method that makes on step
# for the TMC2209 there needs to be a signal duration of minimum 100 ns
#-----------------------------------------------------------------------
    def makeAStep(self):
        GPIO.output(self._pin_step, GPIO.HIGH)
        time.sleep(1/1000/1000)
        GPIO.output(self._pin_step, GPIO.LOW)
        time.sleep(1/1000/1000)

        if(self._loglevel.value >= Loglevel.movement.value):
                print("TMC2209: one step")

#-----------------------------------------------------------------------
# tests the EN, DIR and STEP pin
#-----------------------------------------------------------------------
    def testDirStepEn(self):
        pin_dir_ok = pin_step_ok = pin_en_ok = True
        
        GPIO.output(self._pin_step, GPIO.HIGH)
        GPIO.output(self._pin_dir, GPIO.HIGH)
        GPIO.output(self._pin_en, GPIO.HIGH)
        time.sleep(0.1)
        ioin = self.readIOIN()
        if(not(ioin & reg.io_dir)):
            pin_dir_ok = False
        if(not(ioin & reg.io_step)):
            pin_step_ok = False
        if(not(ioin & reg.io_enn)):
            pin_en_ok = False

        GPIO.output(self._pin_step, GPIO.LOW)
        GPIO.output(self._pin_dir, GPIO.LOW)
        GPIO.output(self._pin_en, GPIO.LOW)
        time.sleep(0.1)
        ioin = self.readIOIN()
        if(ioin & reg.io_dir):
            pin_dir_ok = False
        if(ioin & reg.io_step):
            pin_step_ok = False
        if(ioin & reg.io_enn):
            pin_en_ok = False

        GPIO.output(self._pin_step, GPIO.HIGH)
        GPIO.output(self._pin_dir, GPIO.HIGH)
        GPIO.output(self._pin_en, GPIO.HIGH)
        time.sleep(0.1)
        ioin = self.readIOIN()
        if(not(ioin & reg.io_dir)):
            pin_dir_ok = False
        if(not(ioin & reg.io_step)):
            pin_step_ok = False
        if(not(ioin & reg.io_enn)):
            pin_en_ok = False

        self.setMotorEnabled(False)

        print("---")
        if(pin_dir_ok):
            print("Pin DIR: \tOK")
        else:
            print("Pin DIR: \tnot OK")
        if(pin_step_ok):
            print("Pin STEP: \tOK")
        else:
            print("Pin STEP: \tnot OK")
        if(pin_en_ok):
            print("Pin EN: \tOK")
        else:
            print("Pin EN: \tnot OK")
        print("---")


#-----------------------------------------------------------------------
# test method
#-----------------------------------------------------------------------
    def test(self):
        self.setDirection_pin(1)
        
        for i in range(100):
            self._currentPos += 1
            GPIO.output(self._pin_step, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(self._pin_step, GPIO.LOW)
            time.sleep(0.01)