from .TMC_2209_uart import TMC_UART
from . import TMC_2209_reg as reg
import RPi.GPIO as GPIO
import time
from enum import Enum
import math
import statistics
import threading



class Direction(Enum):
    CCW = 0
    CW = 1


class Loglevel(Enum):
    NONE = 0
    ERROR = 10
    INFO = 20
    DEBUG = 30
    MOVEMENT = 40
    ALL = 100


class MovementAbsRel(Enum):
    ABSOLUTE = 0
    RELATIVE = 1


class MovementPhase(Enum):
    STANDSTILL = 0
    ACCELERATING = 1
    MAXSPEED = 2
    DEACCELERATING = 3

class StopMode(Enum):
    NO = 0
    SOFTSTOP = 1
    HARDSTOP = 2
    




#-----------------------------------------------------------------------
# TMC_2209
#
# this class has two different functions:
# 1. change setting in the TMC-driver via UART
# 2. move the motor via STEP/DIR pins
#-----------------------------------------------------------------------
class TMC_2209:
    
    tmc_uart = None
    _pin_step = -1
    _pin_dir = -1
    _pin_en = -1
    _pin_stallguard = -1

    _direction = True

    _stop = StopMode.NO
    _starttime = 0
    _sg_delay = 0
    _sg_callback = None
    

    _msres = -1
    _steps_per_revolution = 0
    
    _loglevel = Loglevel.INFO
    _logprefix = "TMC2209"

    _current_pos = 0                 # current position of stepper in steps
    _target_pos = 0                  # the target position in steps
    _speed = 0.0                    # the current speed in steps per second
    _max_speed = 1.0                 # the maximum speed in steps per second
    _max_speed_homing = 500           # the maximum speed in steps per second for homing
    _acceleration = 1.0             # the acceleration in steps per second per second
    _acceleration_homing = 10000     # the acceleration in steps per second per second for homing
    _sqrt_twoa = 1.0                # Precomputed sqrt(2*_acceleration)
    _step_interval = 0               # the current interval between two steps
    _min_pulse_width = 1              # minimum allowed pulse with in microseconds
    _last_step_time = 0               # The last step time in microseconds
    _n = 0                          # step counter
    _c0 = 0                         # Initial step size in microseconds
    _cn = 0                         # Last step size in microseconds
    _cmin = 0                       # Min step size in microseconds based on maxSpeed
    _sg_threshold = 100             # threshold for stallguard
    _movement_abs_rel = MovementAbsRel.ABSOLUTE
    _movement_phase = MovementPhase.STANDSTILL

    _movement_thread = None

    _deinit_finished = False
    
    
#-----------------------------------------------------------------------
# constructor
#-----------------------------------------------------------------------
    def __init__(self, pin_en, pin_step=-1, pin_dir=-1, baudrate=115200, serialport="/dev/serial0", driver_address=0, no_uart=False, gpio_mode=GPIO.BCM, loglevel = None):
        self.init(pin_en, pin_step, pin_dir, baudrate, serialport, driver_address, no_uart, gpio_mode, loglevel)


#-----------------------------------------------------------------------
# destructor
#-----------------------------------------------------------------------
    def __del__(self):
        self.deinit()
        

#-----------------------------------------------------------------------
# init function
#-----------------------------------------------------------------------
    def init(self, pin_en, pin_step=-1, pin_dir=-1, baudrate=115200, serialport="/dev/serial0", driver_address=0, no_uart=False, gpio_mode=GPIO.BCM, loglevel = None):
        self.tmc_uart = TMC_UART(serialport, baudrate, driver_address)

        if(loglevel != None):
            self._loglevel = loglevel

        self.log("Init", Loglevel.INFO.value)
        GPIO.setwarnings(False)
        GPIO.setmode(gpio_mode)

        self.log("EN Pin: " + str(pin_en), Loglevel.DEBUG.value)
        self._pin_en = pin_en
        GPIO.setup(self._pin_en, GPIO.OUT, initial=GPIO.HIGH)

        self.log("STEP Pin: " + str(pin_step), Loglevel.DEBUG.value)
        if(pin_step != -1):
            self._pin_step = pin_step
            GPIO.setup(self._pin_step, GPIO.OUT, initial=GPIO.LOW)

        self.log("DIR Pin: " + str(pin_dir), Loglevel.DEBUG.value)
        if(pin_dir != -1):
            self._pin_dir = pin_dir
            GPIO.setup(self._pin_dir, GPIO.OUT, initial=self._direction)

        self.log("GPIO Init finished", Loglevel.INFO.value)
        
        if(not no_uart):
            self.read_steps_per_revolution()
            self.clearGSTAT()

        self.tmc_uart.flush_serial_buffer()
        self.log("Init finished", Loglevel.INFO.value)


#-----------------------------------------------------------------------
# deinit function
#-----------------------------------------------------------------------
    def deinit(self):
        if(self._deinit_finished == False):
            self.log("Deinit", Loglevel.INFO.value)

            self.set_motor_enabled(False)

            self.log("GPIO cleanup")
            if(self._pin_step != -1):
                GPIO.cleanup(self._pin_step)
            if(self._pin_dir != -1):
                GPIO.cleanup(self._pin_dir)
            if(self._pin_en != -1):
                GPIO.cleanup(self._pin_en)
            if(self._pin_stallguard != -1):
                GPIO.remove_event_detect(self._pin_stallguard)
                GPIO.cleanup(self._pin_stallguard)
            
            self.log("Deinit finished", Loglevel.INFO.value)
            self._deinit_finished= True
        else:
            self.log("Deinit already finished", Loglevel.INFO.value)


#-----------------------------------------------------------------------
# set deinitialize to true
#-----------------------------------------------------------------------       
    def set_deinitialize_true(self):
        self._deinit_finished = True
        


#-----------------------------------------------------------------------
# set the loglevel. See the Enum Loglevel
#-----------------------------------------------------------------------       
    def set_loglevel(self, loglevel):
        self._loglevel = loglevel


#-----------------------------------------------------------------------
# logs a message
#-----------------------------------------------------------------------       
    def log(self, message, loglevel=Loglevel.NONE.value):
        if(self._loglevel.value >= loglevel):
            print(self._logprefix+"_"+str(self.tmc_uart.mtr_id)+": "+message)

#-----------------------------------------------------------------------
# set whether the movment should be relative or absolute by default.
# See the Enum MovementAbsoluteRelative
#-----------------------------------------------------------------------       
    def set_movement_abs_rel(self, movement_abs_rel):
        self._movement_abs_rel = movement_abs_rel





#-----------------------------------------------------------------------
# read the register Adress "DRVSTATUS" and prints all current setting
#-----------------------------------------------------------------------
    def readDRVSTATUS(self):
        self.log("---")
        self.log("DRIVER STATUS:")
        drvstatus =self.tmc_uart.read_int(reg.DRVSTATUS)
        self.log(bin(drvstatus), Loglevel.INFO.value)
        if(drvstatus & reg.stst):
            self.log("Info: motor is standing still")
        else:
            self.log("Info: motor is running")

        if(drvstatus & reg.stealth):
            self.log("Info: motor is running on StealthChop")
        else:
            self.log("Info: motor is running on SpreadCycle")

        cs_actual = drvstatus & reg.cs_actual
        cs_actual = cs_actual >> 16
        self.log("CS actual: "+str(cs_actual))

        if(drvstatus & reg.olb):
            self.log("Warning: Open load detected on phase B")
        
        if(drvstatus & reg.ola):
            self.log("Warning: Open load detected on phase A")
        
        if(drvstatus & reg.s2vsb):
            self.log("Error: Short on low-side MOSFET detected on phase B. The driver becomes disabled")

        if(drvstatus & reg.s2vsa):
            self.log("Error: Short on low-side MOSFET detected on phase A. The driver becomes disabled")

        if(drvstatus & reg.s2gb):
            self.log("Error: Short to GND detected on phase B. The driver becomes disabled.")
        
        if(drvstatus & reg.s2ga):
            self.log("Error: Short to GND detected on phase A. The driver becomes disabled.")
        
        if(drvstatus & reg.ot):
            self.log("Error: Driver Overheating!")
        
        if(drvstatus & reg.otpw):
            self.log("Warning: Driver Overheating Prewarning!")
        
        print("---")
        return drvstatus
            

#-----------------------------------------------------------------------
# read the register Adress "GCONF" and prints all current setting
#-----------------------------------------------------------------------
    def readGCONF(self):
        self.log("---")
        self.log("GENERAL CONFIG")
        gconf = self.tmc_uart.read_int(reg.GCONF)
        self.log(bin(gconf), Loglevel.INFO.value)

        if(gconf & reg.i_scale_analog):
            self.log("Driver is using voltage supplied to VREF as current reference")
        else:
            self.log("Driver is using internal reference derived from 5VOUT")
        if(gconf & reg.internal_rsense):
            self.log("Internal sense resistors. Use current supplied into VREF as reference.")
            self.log("VREF pin internally is driven to GND in this mode.")
            self.log("This will most likely destroy your driver!!!")
            raise SystemExit
        else:
            self.log("Operation with external sense resistors")
        if(gconf & reg.en_spreadcycle):
            self.log("SpreadCycle mode enabled")
        else:
            self.log("StealthChop PWM mode enabled")
        if(gconf & reg.shaft):
            self.log("Inverse motor direction")
        else:
            self.log("normal motor direction")
        if(gconf & reg.index_otpw):
            self.log("INDEX pin outputs overtemperature prewarning flag")
        else:
            self.log("INDEX shows the first microstep position of sequencer")
        if(gconf & reg.index_step):
            self.log("INDEX output shows step pulses from internal pulse generator")
        else:
            self.log("INDEX output as selected by index_otpw")
        if(gconf & reg.mstep_reg_select):
            self.log("Microstep resolution selected by MSTEP register")
        else:
            self.log("Microstep resolution selected by pins MS1, MS2")
        
        self.log("---")
        return gconf


#-----------------------------------------------------------------------
# read the register Adress "GSTAT" and prints all current setting
#-----------------------------------------------------------------------
    def readGSTAT(self):
        self.log("---")
        self.log("GSTAT")
        gstat = self.tmc_uart.read_int(reg.GSTAT)
        self.log(bin(gstat), Loglevel.INFO.value)
        if(gstat & reg.reset):
            self.log("The Driver has been reset since the last read access to GSTAT")
        if(gstat & reg.drv_err):
            self.log("The driver has been shut down due to overtemperature or short circuit detection since the last read access")
        if(gstat & reg.uv_cp):
            self.log("Undervoltage on the charge pump. The driver is disabled in this case")
        self.log("---")
        return gstat


#-----------------------------------------------------------------------
# read the register Adress "GSTAT" and prints all current setting
#-----------------------------------------------------------------------
    def clearGSTAT(self):
        self.log("clearing GSTAT", Loglevel.INFO.value)
        gstat = self.tmc_uart.read_int(reg.GSTAT)
        
        gstat = self.tmc_uart.set_bit(gstat, reg.reset)
        gstat = self.tmc_uart.set_bit(gstat, reg.drv_err)
        
        self.tmc_uart.write_reg_check(reg.GSTAT, gstat)

 
#-----------------------------------------------------------------------
# read the register Adress "IOIN" and prints all current setting
#-----------------------------------------------------------------------
    def readIOIN(self):
        self.log("---")
        self.log("INPUTS")
        ioin = self.tmc_uart.read_int(reg.IOIN)
        self.log(bin(ioin), Loglevel.INFO.value)
        if(ioin & reg.io_spread):
            self.log("spread is high")
        else:
            self.log("spread is low")

        if(ioin & reg.io_dir):
            self.log("dir is high")
        else:
            self.log("dir is low")

        if(ioin & reg.io_step):
            self.log("step is high")
        else:
            self.log("step is low")

        if(ioin & reg.io_enn):
            self.log("en is high")
        else:
            self.log("en is low")
        
        self.log("---")
        return ioin


#-----------------------------------------------------------------------
# read the register Adress "CHOPCONF" and prints all current setting
#-----------------------------------------------------------------------
    def readCHOPCONF(self):
        self.log("---")
        self.log("CHOPPER CONTROL")
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        self.log(bin(chopconf), Loglevel.INFO.value)
        
        self.log("native "+str(self.get_microstepping_resolution())+" microstep setting")
        
        if(chopconf & reg.intpol):
            self.log("interpolation to 256 microsteps")
        
        if(chopconf & reg.vsense):
            self.log("1: High sensitivity, low sense resistor voltage")
        else:
            self.log("0: Low sensitivity, high sense resistor voltage")

        self.log("---")
        return chopconf





#-----------------------------------------------------------------------
# enables or disables the motor current output
#-----------------------------------------------------------------------
    def set_motor_enabled(self, en):
        GPIO.output(self._pin_en, not en)
        self.log("Motor output active: {}".format(en), Loglevel.INFO.value)
      

#-----------------------------------------------------------------------
# homes the motor in the given direction using stallguard
# 1. param: DIAG pin
# 2. param: maximum number of revolutions. Can be negative for inverse direction
# 3. param(optional): StallGuard detection threshold
# returns true when homing was successful
#-----------------------------------------------------------------------
    def do_homing(self, diag_pin, revolutions, threshold=None):        
        if(threshold != None):
            self._sg_threshold = threshold
        
        self.log("---", Loglevel.INFO.value)
        self.log("homing", Loglevel.INFO.value)

        self.set_spreadcycle(0)

        self.log("Stallguard threshold:"+str(self._sg_threshold), Loglevel.DEBUG.value)

        self.set_stallguard_callback(diag_pin, self._sg_threshold, self.stop)
        
        homing_failed = self.set_vactual_rpm(30, revolutions=revolutions)

        if(homing_failed):
            self.log("homing failed", Loglevel.ERROR.value)
        else:
            self.log("homing successful",Loglevel.INFO.value)
        
        self._current_pos = 0
        
        self.log("---", Loglevel.INFO.value)
        return not homing_failed
        

#-----------------------------------------------------------------------
# homes the motor in the given direction using stallguard
# old function, uses STEP/DIR 
# 1. param: direction
# 2. param(optional): StallGuard detection threshold
#-----------------------------------------------------------------------
    def do_homing2(self, direction, threshold=None):
        sg_results = []
        
        if(threshold != None):
            self._sg_threshold = threshold
        
        self.log("---", Loglevel.INFO.value)
        self.log("homing", Loglevel.INFO.value)
        
        self.log("Stallguard threshold:"+str(self._sg_threshold), Loglevel.DEBUG.value)
        
        self.set_direction_pin(direction)
        self.set_spreadcycle(0)

        if (direction == 1):
            self._target_pos = self._steps_per_revolution * 10
        else:
            self._target_pos = -self._steps_per_revolution * 10
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self.set_acceleration(10000)
        self.set_max_speed(self._max_speed_homing)
        self.compute_new_speed()


        step_counter=0
        #self.log("Steps per Revolution: "+str(self._steps_per_revolution))
        while (step_counter<self._steps_per_revolution):
            if (self.run_speed()): #returns true, when a step is made
                step_counter += 1
                self.compute_new_speed()
                sg_result = self.get_stallguard_result()
                sg_results.append(sg_result)
                if(len(sg_results)>20):
                    sg_result_average = statistics.mean(sg_results[-6:])
                    if(sg_result_average < self._sg_threshold):
                        break

        if(step_counter<self._steps_per_revolution):
            self.log("homing successful",Loglevel.INFO.value)
            self.log("Stepcounter: "+str(step_counter),Loglevel.DEBUG.value)
            self.log(str(sg_results),Loglevel.DEBUG.value)
            self._current_pos = 0
        else:
            self.log("homing failed", Loglevel.ERROR.value)
            self.log("Stepcounter: "+str(step_counter), Loglevel.DEBUG.value)
            self.log(str(sg_results),Loglevel.DEBUG.value)
        
        self.log("---", Loglevel.INFO.value)
        

#-----------------------------------------------------------------------
# returns the current motor position in microsteps
#-----------------------------------------------------------------------
    def get_current_position(self):
        return self._current_pos


#-----------------------------------------------------------------------
# overwrites the current motor position in microsteps
#-----------------------------------------------------------------------
    def set_current_position(self, new_pos):
        self._current_pos = new_pos

       
#-----------------------------------------------------------------------
# reverses the motor shaft direction
#-----------------------------------------------------------------------
    def reverse_direction_pin(self):
        self._direction = not self._direction
        GPIO.output(self._pin_dir, self.direction)


#-----------------------------------------------------------------------
# sets the motor shaft direction to the given value: 0 = CCW; 1 = CW
#-----------------------------------------------------------------------
    def set_direction_pin(self, direction):
        self._direction = direction
        GPIO.output(self._pin_dir, direction)


#-----------------------------------------------------------------------
# returns the motor shaft direction: 0 = CCW; 1 = CW
#-----------------------------------------------------------------------
    def get_direction_reg(self):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        return (gconf & reg.shaft)


#-----------------------------------------------------------------------
# sets the motor shaft direction to the given value: 0 = CCW; 1 = CW
#-----------------------------------------------------------------------
    def set_direction_reg(self, direction):        
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(direction):
            self.log("write inverse motor direction", Loglevel.INFO.value)
            gconf = self.tmc_uart.set_bit(gconf, reg.shaft)
        else:
            self.log("write normal motor direction", Loglevel.INFO.value)
            gconf = self.tmc_uart.clear_bit(gconf, reg.shaft)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)
  

#-----------------------------------------------------------------------
# return whether Vref (1) or 5V (0) is used for current scale
#-----------------------------------------------------------------------
    def get_iscale_analog(self):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        return (gconf & reg.i_scale_analog)


#-----------------------------------------------------------------------
# sets Vref (1) or 5V (0) for current scale
#-----------------------------------------------------------------------
    def set_iscale_analog(self,en):        
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(en):
            self.log("activated Vref for current scale", Loglevel.INFO.value)
            gconf = self.tmc_uart.set_bit(gconf, reg.i_scale_analog)
        else:
            self.log("activated 5V-out for current scale", Loglevel.INFO.value)
            gconf = self.tmc_uart.clear_bit(gconf, reg.i_scale_analog)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)


#-----------------------------------------------------------------------
# returns which sense resistor voltage is used for current scaling
# 0: Low sensitivity, high sense resistor voltage
# 1: High sensitivity, low sense resistor voltage
#-----------------------------------------------------------------------
    def getvsense(self):
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        return (chopconf & reg.vsense)


#-----------------------------------------------------------------------
# sets which sense resistor voltage is used for current scaling
# 0: Low sensitivity, high sense resistor voltage
# 1: High sensitivity, low sense resistor voltage
#-----------------------------------------------------------------------
    def setvsense(self,en):      
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        if(en):
            self.log("activated High sensitivity, low sense resistor voltage", Loglevel.INFO.value)
            chopconf = self.tmc_uart.set_bit(chopconf, reg.vsense)
        else:
            self.log("activated Low sensitivity, high sense resistor voltage", Loglevel.INFO.value)
            chopconf = self.tmc_uart.clear_bit(chopconf, reg.vsense)
        self.tmc_uart.write_reg_check(reg.CHOPCONF, chopconf)


#-----------------------------------------------------------------------
# returns which sense resistor voltage is used for current scaling
# 0: Low sensitivity, high sense resistor voltage
# 1: High sensitivity, low sense resistor voltage
#-----------------------------------------------------------------------
    def get_internal_rsense(self):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        return (gconf & reg.internal_rsense)


#-----------------------------------------------------------------------
# sets which sense resistor voltage is used for current scaling
# 0: Low sensitivity, high sense resistor voltage
# 1: High sensitivity, low sense resistor voltage
#-----------------------------------------------------------------------
    def set_internal_rsense(self,en):        
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(en):
            self.log("activated internal sense resistors.", Loglevel.INFO.value)
            self.log("VREF pin internally is driven to GND in this mode.", Loglevel.INFO.value)
            self.log("This will most likely destroy your driver!!!", Loglevel.INFO.value)
            raise SystemExit
            gconf = self.tmc_uart.set_bit(gconf, reg.internal_rsense)
        else:
            self.log("activated operation with external sense resistors", Loglevel.INFO.value)
            gconf = self.tmc_uart.clear_bit(gconf, reg.internal_rsense)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)
    

#-----------------------------------------------------------------------
# sets the current scale (CS) for Running and Holding
# and the delay, when to be switched to Holding current
# IHold = 0-31; IRun = 0-31; ihold_delay = 0-15
#-----------------------------------------------------------------------
    def set_irun_ihold(self, IHold, IRun, ihold_delay):
        ihold_irun = 0
        
        ihold_irun = ihold_irun | IHold << 0
        ihold_irun = ihold_irun | IRun << 8
        ihold_irun = ihold_irun | ihold_delay << 16
        self.log("ihold_irun", Loglevel.INFO.value)
        self.log(str(bin(ihold_irun)), Loglevel.INFO.value)

        self.log("writing ihold_irun", Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(reg.IHOLD_IRUN, ihold_irun)


#-----------------------------------------------------------------------
# disables PDN on the UART pin
# 0: PDN_UART controls standstill current reduction
# 1: PDN_UART input function disabled. Set this bit,
# when using the UART interface!
#-----------------------------------------------------------------------
    def set_pdn_disable(self,pdn_disable):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(pdn_disable):
            self.log("enabled PDN_UART", Loglevel.INFO.value)
            gconf = self.tmc_uart.set_bit(gconf, reg.pdn_disable)
        else:
            self.log("disabled PDN_UART", Loglevel.INFO.value)
            gconf = self.tmc_uart.clear_bit(gconf, reg.pdn_disable)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)


#-----------------------------------------------------------------------
# sets the current flow for the motor
# run_current in mA
# check whether Vref is actually 1.2V
#-----------------------------------------------------------------------
    def set_current(self, run_current, hold_current_multiplier = 0.5, hold_current_delay = 10, use_vref=False, Vref = 1.2, pdn_disable = True):
        cs_irun = 0
        rsense = 0.11
        vfs = 0

        if (use_vref):
            self.set_iscale_analog(True)
        else:
            Vref=2.5
            self.set_iscale_analog(False)

        vfs = 0.325 * Vref / 2.5
        cs_irun = 32.0*1.41421*run_current/1000.0*(rsense+0.02)/vfs - 1

        # If Current Scale is too low, turn on high sensitivity VSsense and calculate again
        if(cs_irun < 16):
            self.log("CS too low; switching to VSense True", Loglevel.INFO.value)
            vfs = 0.180 * Vref / 2.5
            cs_irun = 32.0*1.41421*run_current/1000.0*(rsense+0.02)/vfs - 1
            self.setvsense(True)
        else: # If CS >= 16, turn off high_senser
            self.log("CS in range; using VSense False", Loglevel.INFO.value)
            self.setvsense(False)

        cs_irun = min(cs_irun, 31)
        cs_irun = max(cs_irun, 0)

        CS_IHold = hold_current_multiplier * cs_irun

        cs_irun = round(cs_irun)
        CS_IHold = round(CS_IHold)
        hold_current_delay = round(hold_current_delay)

        self.log("cs_irun: " + str(cs_irun), Loglevel.INFO.value)
        self.log("CS_IHold: " + str(CS_IHold), Loglevel.INFO.value)
        self.log("Delay: " + str(hold_current_delay), Loglevel.INFO.value)

        # return (float)(CS+1)/32.0 * (vsense() ? 0.180 : 0.325)/(rsense+0.02) / 1.41421 * 1000;
        run_current_actual = (cs_irun+1)/32.0 * (vfs)/(rsense+0.02) / 1.41421 * 1000
        self.log("actual current: "+str(round(run_current_actual))+" mA", Loglevel.INFO.value)

        self.set_irun_ihold(CS_IHold, cs_irun, hold_current_delay)

        self.set_pdn_disable(pdn_disable)


#-----------------------------------------------------------------------
# return whether spreadcycle (1) is active or stealthchop (0)
#-----------------------------------------------------------------------
    def get_spreadcycle(self):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        return (gconf & reg.en_spreadcycle)


#-----------------------------------------------------------------------
# enables spreadcycle (1) or stealthchop (0)
#-----------------------------------------------------------------------
    def set_spreadcycle(self,en_spread):
        gconf = self.tmc_uart.read_int(reg.GCONF)
        if(en_spread):
            self.log("activated Spreadcycle", Loglevel.INFO.value)
            gconf = self.tmc_uart.set_bit(gconf, reg.en_spreadcycle)
        else:
            self.log("activated Stealthchop", Loglevel.INFO.value)
            gconf = self.tmc_uart.clear_bit(gconf, reg.en_spreadcycle)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)


#-----------------------------------------------------------------------
# return whether the tmc inbuilt interpolation is active
#-----------------------------------------------------------------------
    def get_interpolation(self):
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        if(chopconf & reg.intpol):
            return True
        else:
            return False

    
#-----------------------------------------------------------------------
# enables the tmc inbuilt interpolation of the steps to 256 microsteps
#-----------------------------------------------------------------------
    def set_interpolation(self, en):
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)

        if(en):
            chopconf = self.tmc_uart.set_bit(chopconf, reg.intpol)
        else:
            chopconf = self.tmc_uart.clear_bit(chopconf, reg.intpol)

        self.log("writing microstep interpolation setting: "+str(en), Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(reg.CHOPCONF, chopconf)


#-----------------------------------------------------------------------
# returns the current native microstep resolution (1-256)
#-----------------------------------------------------------------------
    def get_microstepping_resolution(self):
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        
        msresdezimal = chopconf & (reg.msres0 | reg.msres1 | reg.msres2 | reg.msres3)
        
        msresdezimal = msresdezimal >> 24
        msresdezimal = 8 - msresdezimal
                
        self._msres = int(math.pow(2, msresdezimal))
        
        return self._msres


#-----------------------------------------------------------------------
# sets the current native microstep resolution (1,2,4,8,16,32,64,128,256)
#-----------------------------------------------------------------------
    def set_microstepping_resolution(self, msres):      
        chopconf = self.tmc_uart.read_int(reg.CHOPCONF)
        chopconf = chopconf & (~reg.msres0 | ~reg.msres1 | ~reg.msres2 | ~reg.msres3) #setting all bits to zero
        msresdezimal = int(math.log(msres, 2))
        msresdezimal = 8 - msresdezimal
        chopconf = int(chopconf) & int(4043309055)
        chopconf = chopconf | msresdezimal <<24
        
        self.log("writing "+str(msres)+" microstep setting", Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(reg.CHOPCONF, chopconf)
        
        self.set_mstep_resolution_reg_select(True)

        self.read_steps_per_revolution()

        return True
        

#-----------------------------------------------------------------------
# sets the register bit "mstep_reg_select" to 1 or 0 depending to the given value.
# this is needed to set the microstep resolution via UART
# this method is called by "set_microstepping_resolution"
#-----------------------------------------------------------------------
    def set_mstep_resolution_reg_select(self, en):                  
        gconf = self.tmc_uart.read_int(reg.GCONF)
        
        if(en == True):
            gconf = self.tmc_uart.set_bit(gconf, reg.mstep_reg_select)
        else:
            gconf = self.tmc_uart.clear_bit(gconf, reg.mstep_reg_select)

        self.log("writing MStep Reg Select: "+str(en), Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(reg.GCONF, gconf)
        

#-----------------------------------------------------------------------
# returns how many steps are needed for one revolution
#-----------------------------------------------------------------------
    def read_steps_per_revolution(self):
        self._steps_per_revolution = 200*self.get_microstepping_resolution()
        return self._steps_per_revolution


#-----------------------------------------------------------------------
# returns how many steps are needed for one revolution
#-----------------------------------------------------------------------
    def get_steps_per_revolution(self):
        return self._steps_per_revolution


#-----------------------------------------------------------------------
# reads the interface transmission counter from the tmc register
# this value is increased on every succesfull write access
# can be used to verify a write access
#-----------------------------------------------------------------------
    def get_interface_transmission_counter(self):
        ifcnt = self.tmc_uart.read_int(reg.IFCNT)
        self.log("Interface Transmission Counter: "+str(ifcnt), Loglevel.INFO.value)
        return ifcnt


#-----------------------------------------------------------------------
# return the current stallguard result
# its will be calculated with every fullstep
# higher values means a lower motor load
#-----------------------------------------------------------------------
    def get_tstep(self):
        tstep = self.tmc_uart.read_int(reg.TSTEP)
        return tstep


#-----------------------------------------------------------------------
# sets the register bit "VACTUAL" to to a given value
# VACTUAL allows moving the motor by UART control.
# It gives the motor velocity in +-(2^23)-1 [μsteps / t]
# 0: Normal operation. Driver reacts to STEP input
#-----------------------------------------------------------------------
    def set_vactual(self, vactual, duration=0, acceleration=0, show_stallguard_result=False, show_tstep=False):
        self._stop = StopMode.NO
        current_vactual = 0
        sleeptime = 0.05
        if(vactual<0):
            acceleration = -acceleration

        if(duration != 0):
            self.log("vactual: "+str(vactual)+" for "+str(duration)+" sec", Loglevel.INFO.value)
        else:
            self.log("vactual: "+str(vactual), Loglevel.INFO.value)
        self.log(str(bin(vactual)), Loglevel.INFO.value)

        self.log("writing vactual", Loglevel.INFO.value)
        if(acceleration == 0):
            self.tmc_uart.write_reg_check(reg.VACTUAL, vactual)

        if(duration != 0):
            self._starttime = time.time()
            current_time = time.time()
            while((current_time < self._starttime+duration)):
                if(self._stop == StopMode.HARDSTOP):
                    break
                if(acceleration != 0):
                    time_to_stop = self._starttime+duration-abs(current_vactual/acceleration)
                    if(self._stop == StopMode.SOFTSTOP):
                        time_to_stop = current_time-1
                    #self.log("cur: "+str(current_time)+ "\t| stop: "+str(time_to_stop)+ "\t| vac: "+str(current_vactual)+ "\t| acc: "+str(acceleration), Loglevel.INFO.value)
                if(acceleration != 0 and current_time > time_to_stop):
                    current_vactual -= acceleration*sleeptime
                    self.tmc_uart.write_reg_check(reg.VACTUAL, int(round(current_vactual)))
                    time.sleep(sleeptime)
                elif(acceleration != 0 and abs(current_vactual)<abs(vactual)):
                    current_vactual += acceleration*sleeptime
                    #self.log("current_vactual: "+str(int(round(current_vactual))), Loglevel.INFO.value)
                    self.tmc_uart.write_reg_check(reg.VACTUAL, int(round(current_vactual)))
                    time.sleep(sleeptime)
                if(show_stallguard_result):
                    self.log("StallGuard result: "+str(self.get_stallguard_result()), Loglevel.INFO.value)
                    time.sleep(0.1)
                if(show_tstep):
                    self.log("TStep result: "+str(self.get_tstep()), Loglevel.INFO.value)
                    time.sleep(0.1)
                current_time = time.time()
            self.tmc_uart.write_reg_check(reg.VACTUAL, 0)
            return self._stop


#-----------------------------------------------------------------------
# converts the rps parameter to a vactual value which represents
# rotation speed in revolutions per second
# With internal oscillator:
# VACTUAL[2209] = v[Hz] / 0.715Hz
#-----------------------------------------------------------------------
    def set_vactual_rps(self, rps, duration=0, revolutions=0, acceleration=0):
        vactual = rps/0.715*self._steps_per_revolution
        if(revolutions!=0):
            duration = abs(revolutions/rps)
        if(revolutions<0):
            vactual = -vactual
        return self.set_vactual(int(round(vactual)), duration, acceleration=acceleration)


#-----------------------------------------------------------------------
# converts the rps parameter to a vactual value which represents
# rotation speed in revolutions per minute
#-----------------------------------------------------------------------
    def set_vactual_rpm(self, rpm, duration=0, revolutions=0, acceleration=0):
        return self.set_vactual_rps(rpm/60, duration, revolutions, acceleration)


#-----------------------------------------------------------------------
# return the current stallguard result
# its will be calculated with every fullstep
# higher values means a lower motor load
#-----------------------------------------------------------------------
    def get_stallguard_result(self):
        sg_result = self.tmc_uart.read_int(reg.SG_RESULT)
        return sg_result


#-----------------------------------------------------------------------
# sets the register bit "SGTHRS" to to a given value
# this is needed for the stallguard interrupt callback
# SG_RESULT becomes compared to the double of this threshold.
# SG_RESULT ≤ SGTHRS*2
#-----------------------------------------------------------------------
    def set_stallguard_threshold(self, threshold):

        self.log("sgthrs", Loglevel.INFO.value)
        self.log(str(bin(threshold)), Loglevel.INFO.value)

        self.log("writing sgthrs", Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(reg.SGTHRS, threshold)


#-----------------------------------------------------------------------
# This  is  the  lower  threshold  velocity  for  switching  
# on  smart energy CoolStep and StallGuard to DIAG output. (unsigned)
#-----------------------------------------------------------------------
    def set_coolstep_threshold(self, threshold):

        self.log("tcoolthrs", Loglevel.INFO.value)
        self.log(str(bin(threshold)), Loglevel.INFO.value)

        self.log("writing tcoolthrs", Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(reg.TCOOLTHRS, threshold)


#-----------------------------------------------------------------------
# set a function to call back, when the driver detects a stall 
# via stallguard
# high value on the diag pin can also mean a driver error
#-----------------------------------------------------------------------
    def set_stallguard_callback(self, pin_stallguard, threshold, callback, min_speed = 2000, ignore_delay = 0):
        self.log("setup stallguard callback on GPIO"+str(pin_stallguard), Loglevel.INFO.value)
        self.log("StallGuard Threshold: "+str(threshold)+"\tminimum Speed: "+str(min_speed), Loglevel.INFO.value)

        self.set_stallguard_threshold(threshold)
        self.set_coolstep_threshold(min_speed)
        self._sg_delay = ignore_delay
        self._sg_callback = callback
        self._pin_stallguard = pin_stallguard
        
        GPIO.setup(self._pin_stallguard, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
        GPIO.add_event_detect(self._pin_stallguard, GPIO.RISING, callback=self.stallguard_callback, bouncetime=300) 


#-----------------------------------------------------------------------
# the callback function for StallGuard.
# only checks whether the duration of the current movement is longer than
# _sg_delay and then calls the actual callback
#-----------------------------------------------------------------------
    def stallguard_callback(self, channel):
        if(self._sg_callback == None):
            self.log("StallGuard callback is None", Loglevel.DEBUG.value)
            return
        if(time.time()<=self._starttime+self._sg_delay and self._sg_delay != 0):
            return
        self._sg_callback(channel)




#-----------------------------------------------------------------------
# returns the current Microstep counter.
# Indicates actual position in the microstep table for CUR_A
#-----------------------------------------------------------------------
    def get_microstep_counter(self):
        mscnt = self.tmc_uart.read_int(reg.MSCNT)
        return mscnt


#-----------------------------------------------------------------------
# returns the current Microstep counter.
# Indicates actual position in the microstep table for CUR_A
#-----------------------------------------------------------------------
    def get_microstep_counter_in_steps(self, offset=0):
        step = (self.get_microstep_counter()-64)*(self._msres*4)/1024
        step = (4*self._msres)-step-1
        step = round(step)
        return step+offset


#-----------------------------------------------------------------------
# sets the maximum motor speed in µsteps per second
#-----------------------------------------------------------------------
    def set_max_speed(self, speed):
        if (speed < 0.0):
           speed = -speed
        if (self._max_speed != speed):
            self._max_speed = speed
            self._cmin = 1000000.0 / speed
            # Recompute _n from current speed and adjust speed if accelerating or cruising
            if (self._n > 0):
                self._n = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
                self.compute_new_speed()


#-----------------------------------------------------------------------
# sets the maximum motor speed in fullsteps per second
#-----------------------------------------------------------------------
    def set_max_speed_fullstep(self, speed):
        self.set_max_speed(speed*self._msres)


#-----------------------------------------------------------------------
# returns the maximum motor speed in steps per second
#-----------------------------------------------------------------------
    def get_max_speed(self):
        return self._max_speed


#-----------------------------------------------------------------------
# sets the motor acceleration/decceleration in µsteps per sec per sec
#-----------------------------------------------------------------------
    def set_acceleration(self, acceleration):
        if (acceleration == 0.0):
            return
        acceleration = abs(acceleration)
        if (self._acceleration != acceleration):
            # Recompute _n per Equation 17
            self._n = self._n * (self._acceleration / acceleration)
            # New c0 per Equation 7, with correction per Equation 15
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
            self._acceleration = acceleration
            self.compute_new_speed()

#-----------------------------------------------------------------------
# sets the motor acceleration/decceleration in fullsteps per sec per sec
#-----------------------------------------------------------------------
    def set_acceleration_fullstep(self, acceleration):
        self.set_acceleration(acceleration*self._msres)


#-----------------------------------------------------------------------
# returns the motor acceleration/decceleration in steps per sec per sec
#-----------------------------------------------------------------------
    def get_acceleration(self):
        return self._acceleration


#-----------------------------------------------------------------------
# stop the current movement
#-----------------------------------------------------------------------
    def stop(self, stop_mode = StopMode.HARDSTOP):
        self._stop = stop_mode


#-----------------------------------------------------------------------
# return the current Movement Phase
#-----------------------------------------------------------------------
    def get_movement_phase(self):
        return self._movement_phase


#-----------------------------------------------------------------------
# runs the motor to the given position.
# with acceleration and deceleration
# blocks the code until finished or stopped from a different thread!
# returns true when the movement if finshed normally and false,
# when the movement was stopped
#-----------------------------------------------------------------------
    def run_to_position_steps(self, steps, movement_abs_rel = None):
        if(movement_abs_rel == None):
            movement_abs_rel = self._movement_abs_rel

        if(movement_abs_rel == MovementAbsRel.RELATIVE):
            self._target_pos = self._current_pos + steps
        else:
            self._target_pos = steps

        self._stop = StopMode.NO
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self.compute_new_speed()
        while (self.run()): #returns false, when target position is reached
             if(self._stop == StopMode.HARDSTOP):
                break
            
        self._movement_phase = MovementPhase.STANDSTILL
        return self._stop


#-----------------------------------------------------------------------
# runs the motor to the given position.
# with acceleration and deceleration
# blocks the code until finished!
#-----------------------------------------------------------------------
    def run_to_position_revolutions(self, revolutions, movement_absolute_relative = None):
        return self.run_to_position_steps(round(revolutions * self._steps_per_revolution), movement_absolute_relative)


#-----------------------------------------------------------------------
# runs the motor to the given position.
# with acceleration and deceleration
# does not block the code
# returns true when the movement if finshed normally and false,
# when the movement was stopped
#-----------------------------------------------------------------------
    def run_to_position_steps_threaded(self, steps, movement_abs_rel = None):
        self._movement_thread = threading.Thread(target=self.run_to_position_steps, args=(steps, movement_abs_rel))
        self._movement_thread.start()


#-----------------------------------------------------------------------
# runs the motor to the given position.
# with acceleration and deceleration
# does not block the code
#-----------------------------------------------------------------------
    def run_to_position_revolutions_threaded(self, revolutions, movement_absolute_relative = None):
        return self.run_to_position_steps_threaded(round(revolutions * self._steps_per_revolution), movement_absolute_relative)


#-----------------------------------------------------------------------
# wait for the motor to finish the movement,
# if startet threaded
# returns true when the movement if finshed normally and false,
# when the movement was stopped
#-----------------------------------------------------------------------
    def wait_for_movement_finished_threaded(self):
        self._movement_thread.join()
        return self._stop

#-----------------------------------------------------------------------
# calculates a new speed if a speed was made
# returns true if the target position is reached
# should not be called from outside!
#-----------------------------------------------------------------------
    def run(self):
        if (self.run_speed()): #returns true, when a step is made
            self.compute_new_speed()
            #print(self.get_stallguard_result())
            #print(self.get_tstep())
        return (self._speed != 0.0 and self.distance_to_go() != 0)


#-----------------------------------------------------------------------
# returns the remaining distance the motor should run
#-----------------------------------------------------------------------
    def distance_to_go(self):
        return self._target_pos - self._current_pos


#-----------------------------------------------------------------------
# returns the calculated current speed depending on the acceleration
# this code is based on: 
# "Generate stepper-motor speed profiles in real time" by David Austin
#
# https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
# https://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
#-----------------------------------------------------------------------
    def compute_new_speed(self):
        distance_to = self.distance_to_go() # +ve is clockwise from curent location
        steps_to_stop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
        if ((distance_to == 0 and steps_to_stop <= 2) or (self._stop == StopMode.SOFTSTOP and steps_to_stop <= 1)):
            # We are at the target and its time to stop
            self._step_interval = 0
            self._speed = 0.0
            self._n = 0
            self._movement_phase = MovementPhase.STANDSTILL
            self.log("time to stop", Loglevel.MOVEMENT.value)
            return
        
        if (distance_to > 0):
            # We are anticlockwise from the target
            # Need to go clockwise from here, maybe decelerate now
            if (self._n > 0):
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((steps_to_stop >= distance_to) or self._direction == Direction.CCW or self._stop == StopMode.SOFTSTOP):
                    self._n = -steps_to_stop # Start deceleration
                    self._movement_phase = MovementPhase.DEACCELERATING
            elif (self._n < 0):
                # Currently decelerating, need to accel again?
                if ((steps_to_stop < distance_to) and self._direction == Direction.CW):
                    self._n = -self._n # Start accceleration
                    self._movement_phase = MovementPhase.ACCELERATING
        elif (distance_to < 0):
            # We are clockwise from the target
            # Need to go anticlockwise from here, maybe decelerate
            if (self._n > 0):
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((steps_to_stop >= -distance_to) or self._direction == Direction.CW or self._stop == StopMode.SOFTSTOP):
                    self._n = -steps_to_stop # Start deceleration
                    self._movement_phase = MovementPhase.DEACCELERATING
            elif (self._n < 0):
                # Currently decelerating, need to accel again?
                if ((steps_to_stop < -distance_to) and self._direction == Direction.CCW):
                    self._n = -self._n # Start accceleration
                    self._movement_phase = MovementPhase.ACCELERATING
        # Need to accelerate or decelerate
        if (self._n == 0):
            # First step from stopped
            self._cn = self._c0
            GPIO.output(self._pin_step, GPIO.LOW)
            #self.log("distance to: " + str(distance_to))
            if(distance_to > 0):
                self.set_direction_pin(1)
                self.log("going CW", Loglevel.MOVEMENT.value)
            else:
                self.set_direction_pin(0)
                self.log("going CCW", Loglevel.MOVEMENT.value)
            self._movement_phase = MovementPhase.ACCELERATING
        else:
            # Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1)) # Equation 13
            self._cn = max(self._cn, self._cmin)
            if(self._cn == self._cmin):
                self._movement_phase = MovementPhase.MAXSPEED
        self._n += 1
        self._step_interval = self._cn
        self._speed = 1000000.0 / self._cn
        if (self._direction == 0):
            self._speed = -self._speed


#-----------------------------------------------------------------------
# this methods does the actual steps with the current speed
#-----------------------------------------------------------------------
    def run_speed(self):
        # Dont do anything unless we actually have a step interval
        if (not self._step_interval):
            return False
        
        curtime = time.time_ns()/1000
        
        #self.log("current time: " + str(curtime))
        #self.log("last st time: " + str(self._last_step_time))
        
        if (curtime - self._last_step_time >= self._step_interval):

            if (self._direction == 1): # Clockwise
                self._current_pos += 1
            else: # Anticlockwise 
                self._current_pos -= 1
            self.make_a_step()
            
            self._last_step_time = curtime # Caution: does not account for costs in step()
            return True
        else:
            return False



#-----------------------------------------------------------------------
# method that makes on step
# for the TMC2209 there needs to be a signal duration of minimum 100 ns
#-----------------------------------------------------------------------
    def make_a_step(self):
        GPIO.output(self._pin_step, GPIO.HIGH)
        time.sleep(1/1000/1000)
        GPIO.output(self._pin_step, GPIO.LOW)
        time.sleep(1/1000/1000)

        self.log("one step", Loglevel.MOVEMENT.value)


#-----------------------------------------------------------------------
# tests the EN, DIR and STEP pin
# this sets the EN, DIR and STEP pin to HIGH, LOW and HIGH
# and checks the IOIN Register of the TMC meanwhile
#-----------------------------------------------------------------------
    def test_dir_step_en(self):
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

        self.set_motor_enabled(False)

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
    def test_step(self):
        self.set_direction_pin(1)
        
        for i in range(100):
            self._current_pos += 1
            GPIO.output(self._pin_step, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(self._pin_step, GPIO.LOW)
            time.sleep(0.01)

#-----------------------------------------------------------------------
# test method
#-----------------------------------------------------------------------
    def test_uart(self):
        self.log("---")
        self.log("TEST UART")
        result = self.tmc_uart.test_uart(reg.IOIN)

        snd = result[0]
        rtn = result[1]

        self.log("length snd: "+str(len(snd)), Loglevel.DEBUG.value)
        self.log("length rtn: "+str(len(rtn)), Loglevel.DEBUG.value)

        if(len(rtn)==12):
            self.log("the Raspberry Pi received the sended bits and the answer from the TMC", Loglevel.INFO.value)
        elif(len(rtn)==4):
            self.log("the Raspberry Pi received only the sended bits", Loglevel.INFO.value)
        elif(len(rtn)==0):
            self.log("the Raspberry Pi did not receive anything", Loglevel.INFO.value)
        else:
            self.log("the Raspberry Pi received an unexpected amount of bits: "+str(len(rtn)), Loglevel.INFO.value)

        if(snd[0:4] == rtn[0:4]):
            self.log("the Raspberry Pi received exactly the bits it has send. the first 4 bits are the same", Loglevel.INFO.value)
        else:
            self.log("the Raspberry Pi did not received the bits it has send. the first 4 bits are different", Loglevel.INFO.value)


        self.log("complete", Loglevel.DEBUG.value)
        self.log(str(snd.hex()), Loglevel.DEBUG.value)
        self.log(str(rtn.hex()), Loglevel.DEBUG.value)

        self.log("just the first 4 bits", Loglevel.DEBUG.value)
        self.log(str(snd[0:4].hex()), Loglevel.DEBUG.value)
        self.log(str(rtn[0:4].hex()), Loglevel.DEBUG.value)

        
        self.log("---")
        return True
