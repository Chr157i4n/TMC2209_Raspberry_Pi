#pylint: disable=invalid-name
#pylint: disable=import-error
#pylint: disable=too-many-lines
#pylint: disable=too-many-arguments
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=too-many-instance-attributes
"""
TMC_2209 stepper driver module
"""

import time
from enum import Enum
import math
import statistics
import threading
from RPi import GPIO
from .TMC_2209_uart import TMC_UART as tmc_uart
from . import TMC_2209_reg as tmc_reg
from . import TMC_2209_math as tmc_math



class Direction(Enum):
    """movement direction of the motor"""
    CCW = 0
    CW = 1


class Loglevel(Enum):
    """loglevel"""
    NONE = 0
    ERROR = 10
    INFO = 20
    DEBUG = 30
    MOVEMENT = 40
    ALL = 100


class MovementAbsRel(Enum):
    """movement absolute or relative"""
    ABSOLUTE = 0
    RELATIVE = 1


class MovementPhase(Enum):
    """movement phase"""
    STANDSTILL = 0
    ACCELERATING = 1
    MAXSPEED = 2
    DECELERATING = 3


class StopMode(Enum):
    """stopmode"""
    NO = 0
    SOFTSTOP = 1
    HARDSTOP = 2



class TMC_2209:
    """
    TMC_2209

    this class has two different functions:
    1. change setting in the TMC-driver via UART
    2. move the motor via STEP/DIR pins
    """

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
    _steps_per_rev = 0
    _fullsteps_per_rev = 200

    _loglevel = Loglevel.INFO
    _logprefix = "TMC2209"

    _current_pos = 0                 # current position of stepper in steps
    _target_pos = 0                  # the target position in steps
    _speed = 0.0                    # the current speed in steps per second
    _max_speed = 1.0                 # the maximum speed in steps per second
    _max_speed_homing = 200           # the maximum speed in steps per second for homing
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



    def __init__(self, pin_en, pin_step=-1, pin_dir=-1, baudrate=115200, serialport="/dev/serial0",
                 driver_address=0, no_uart=False, gpio_mode=GPIO.BCM, loglevel = None):
        """
        constructor
        """
        self.init(pin_en, pin_step, pin_dir, baudrate, serialport, driver_address, no_uart,
                  gpio_mode, loglevel)



    def __del__(self):
        """
        destructor
        """
        self.deinit()



    def init(self, pin_en, pin_step=-1, pin_dir=-1, baudrate=115200, serialport="/dev/serial0",
    driver_address=0, no_uart=False, gpio_mode=GPIO.BCM, loglevel = None):
        """
        init function
        
            Parameters:
                pin_en (int): Pin number of EN pin
                pin_step (int): Pin number of STEP pin
                pin_dir (int): Pin number of DIR pin
                baudrate (int): baudrate exp: 9600, 115200
                serialport (string): win: 'COM1'; linux: 'dev/serial0'
                driver_address (int): 0 - 3
                no_uart=False (bool): skip UART init, if only STEP/DIR is used
                gpio_mode (enum): numbering system for the GPIO pins
                loglevel (enum): level for which to log
        """
        self.tmc_uart = tmc_uart(serialport, baudrate, driver_address)

        if loglevel is not None:
            self._loglevel = loglevel

        self.log("Init", Loglevel.INFO.value)
        GPIO.setwarnings(False)
        GPIO.setmode(gpio_mode)

        self.log("EN Pin: " + str(pin_en), Loglevel.DEBUG.value)
        self._pin_en = pin_en
        GPIO.setup(self._pin_en, GPIO.OUT, initial=GPIO.HIGH)

        self.log("STEP Pin: " + str(pin_step), Loglevel.DEBUG.value)
        if pin_step != -1:
            self._pin_step = pin_step
            GPIO.setup(self._pin_step, GPIO.OUT, initial=GPIO.LOW)

        self.log("DIR Pin: " + str(pin_dir), Loglevel.DEBUG.value)
        if pin_dir != -1:
            self._pin_dir = pin_dir
            GPIO.setup(self._pin_dir, GPIO.OUT, initial=self._direction)

        self.log("GPIO Init finished", Loglevel.INFO.value)

        if not no_uart:
            self.read_steps_per_rev()
            self.clearGSTAT()

        self.tmc_uart.flush_serial_buffer()
        self.log("Init finished", Loglevel.INFO.value)



    def deinit(self):
        """
        deinit function
        """
        if self._deinit_finished is False:
            self.log("Deinit", Loglevel.INFO.value)

            self.set_motor_enabled(False)

            self.log("GPIO cleanup")
            if self._pin_step != -1:
                GPIO.cleanup(self._pin_step)
            if self._pin_dir != -1:
                GPIO.cleanup(self._pin_dir)
            if self._pin_en != -1:
                GPIO.cleanup(self._pin_en)
            if self._pin_stallguard != -1:
                GPIO.remove_event_detect(self._pin_stallguard)
                GPIO.cleanup(self._pin_stallguard)

            self.log("Deinit finished", Loglevel.INFO.value)
            self._deinit_finished= True
        else:
            self.log("Deinit already finished", Loglevel.INFO.value)



    def set_deinitialize_true(self):
        """
        set deinitialize to true
        """
        self._deinit_finished = True



    def set_loglevel(self, loglevel):
        """
        set the loglevel. See the Enum Loglevel

            Parameters:
                loglevel (enum): level for which to log
        """
        self._loglevel = loglevel



    def log(self, message, loglevel=Loglevel.NONE.value):
        """
        logs a message

            Parameters:
                message (string): message to log
                loglevel (enum): level for which to log
        """
        if self._loglevel.value >= loglevel:
            print(self._logprefix+"_"+str(self.tmc_uart.mtr_id)+": "+message)



    def set_movement_abs_rel(self, movement_abs_rel):
        """
        set whether the movment should be relative or absolute by default.
        See the Enum MovementAbsoluteRelative

            Paramters:
                movement_abs_rel (enum): whether the movment should be relative or absolute
        """
        self._movement_abs_rel = movement_abs_rel



    def readDRVSTATUS(self):
        """
        read the register Adress "DRV_STATUS" and prints all current setting

            Returns:
                drvstatus (int): 32bit DRV_STATUS Register

        """
        self.log("---")
        self.log("DRIVER STATUS:")
        drvstatus =self.tmc_uart.read_int(tmc_reg.DRVSTATUS)
        self.log(bin(drvstatus), Loglevel.INFO.value)
        if drvstatus & tmc_reg.stst:
            self.log("Info: motor is standing still")
        else:
            self.log("Info: motor is running")

        if drvstatus & tmc_reg.stealth:
            self.log("Info: motor is running on StealthChop")
        else:
            self.log("Info: motor is running on SpreadCycle")

        cs_actual = drvstatus & tmc_reg.cs_actual
        cs_actual = cs_actual >> 16
        self.log("CS actual: "+str(cs_actual))

        if drvstatus & tmc_reg.olb:
            self.log("Warning: Open load detected on phase B")

        if drvstatus & tmc_reg.ola:
            self.log("Warning: Open load detected on phase A")

        if drvstatus & tmc_reg.s2vsb:
            self.log("""Error: Short on low-side MOSFET detected on phase B.
                     The driver becomes disabled""")

        if drvstatus & tmc_reg.s2vsa:
            self.log("""Error: Short on low-side MOSFET detected on phase A.
                     The driver becomes disabled""")

        if drvstatus & tmc_reg.s2gb:
            self.log("Error: Short to GND detected on phase B. The driver becomes disabled.")

        if drvstatus & tmc_reg.s2ga:
            self.log("Error: Short to GND detected on phase A. The driver becomes disabled.")

        if drvstatus & tmc_reg.ot:
            self.log("Error: Driver Overheating!")

        if drvstatus & tmc_reg.otpw:
            self.log("Warning: Driver Overheating Prewarning!")

        self.log("---")
        return drvstatus



    def readGCONF(self):
        """
        read the register Adress "GCONF" and prints all current setting

            Returns:
                gconf (int): 10bit GCONF Register
        """
        self.log("---")
        self.log("GENERAL CONFIG")
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        self.log(bin(gconf), Loglevel.INFO.value)

        if gconf & tmc_reg.i_scale_analog:
            self.log("Driver is using voltage supplied to VREF as current reference")
        else:
            self.log("Driver is using internal reference derived from 5VOUT")
        if gconf & tmc_reg.internal_rsense:
            self.log("Internal sense resistors. Use current supplied into VREF as reference.")
            self.log("VREF pin internally is driven to GND in this mode.")
            self.log("This will most likely destroy your driver!!!")
            raise SystemExit
        self.log("Operation with external sense resistors")
        if gconf & tmc_reg.en_spreadcycle:
            self.log("SpreadCycle mode enabled")
        else:
            self.log("StealthChop PWM mode enabled")
        if gconf & tmc_reg.shaft:
            self.log("Inverse motor direction")
        else:
            self.log("normal motor direction")
        if gconf & tmc_reg.index_otpw:
            self.log("INDEX pin outputs overtemperature prewarning flag")
        else:
            self.log("INDEX shows the first microstep position of sequencer")
        if gconf & tmc_reg.index_step:
            self.log("INDEX output shows step pulses from internal pulse generator")
        else:
            self.log("INDEX output as selected by index_otpw")
        if gconf & tmc_reg.mstep_reg_select:
            self.log("Microstep resolution selected by MSTEP register")
        else:
            self.log("Microstep resolution selected by pins MS1, MS2")

        self.log("---")
        return gconf



    def readGSTAT(self):
        """
        read the register Adress "GSTAT" and prints all current setting

            Returns:
                gstat (int): 3bit GSTAT Register
        """
        self.log("---")
        self.log("GSTAT")
        gstat = self.tmc_uart.read_int(tmc_reg.GSTAT)
        self.log(bin(gstat), Loglevel.INFO.value)
        if gstat & tmc_reg.reset:
            self.log("The Driver has been reset since the last read access to GSTAT")
        if gstat & tmc_reg.drv_err:
            self.log("""The driver has been shut down due to overtemperature or
                     short circuit detection since the last read access""")
        if gstat & tmc_reg.uv_cp:
            self.log("Undervoltage on the charge pump. The driver is disabled in this case")
        self.log("---")
        return gstat



    def clearGSTAT(self):
        """
        read the register Adress "GSTAT" and prints all current setting
        """
        self.log("clearing GSTAT", Loglevel.INFO.value)
        gstat = self.tmc_uart.read_int(tmc_reg.GSTAT)

        gstat = self.tmc_uart.set_bit(gstat, tmc_reg.reset)
        gstat = self.tmc_uart.set_bit(gstat, tmc_reg.drv_err)

        self.tmc_uart.write_reg_check(tmc_reg.GSTAT, gstat)



    def readIOIN(self):
        """
        read the register Adress "IOIN" and prints all current setting

            Returns:
                ioin (int): 10+8bit IOIN Register
        """
        self.log("---")
        self.log("INPUTS")
        ioin = self.tmc_uart.read_int(tmc_reg.IOIN)
        self.log(bin(ioin), Loglevel.INFO.value)
        if ioin & tmc_reg.io_spread:
            self.log("spread is high")
        else:
            self.log("spread is low")

        if ioin & tmc_reg.io_dir:
            self.log("dir is high")
        else:
            self.log("dir is low")

        if ioin & tmc_reg.io_step:
            self.log("step is high")
        else:
            self.log("step is low")

        if ioin & tmc_reg.io_enn:
            self.log("en is high")
        else:
            self.log("en is low")

        self.log("---")
        return ioin



    def readCHOPCONF(self):
        """
        read the register Adress "CHOPCONF" and prints all current setting

            Returns:
                chopconf (int): 3bit CHOPCONF Register
        """
        self.log("---")
        self.log("CHOPPER CONTROL")
        chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
        self.log(bin(chopconf), Loglevel.INFO.value)

        self.log("native "+str(self.get_microstepping_resolution())+" microstep setting")

        if chopconf & tmc_reg.intpol:
            self.log("interpolation to 256 microsteps")

        if chopconf & tmc_reg.vsense:
            self.log("1: High sensitivity, low sense resistor voltage")
        else:
            self.log("0: Low sensitivity, high sense resistor voltage")

        self.log("---")
        return chopconf



    def set_motor_enabled(self, en):
        """
        enables or disables the motor current output

            Parameters:
                en (bool): whether the motor current output should be enabled
        """
        GPIO.output(self._pin_en, not en)
        self.log(f"Motor output active: {en}", Loglevel.INFO.value)



    def do_homing(self, diag_pin, revolutions = 10, threshold = None, speed_rpm = None):
        """
        homes the motor in the given direction using stallguard

            Parameters
                diag_pin (int): DIAG pin number
                revolutions (int): max number of revolutions. Can be negative for inverse direction
                threshold (int): optional; StallGuard detection threshold
                speed_rpm (float): optional; speed in revolutions per minute

            Returns:
                not homing_failed (bool): true when homing was successful
        """
        if threshold is not None:
            self._sg_threshold = threshold
        if speed_rpm is None:
            speed_rpm = tmc_math.steps_to_rps(self._max_speed_homing, self._steps_per_rev)*60

        self.log("---", Loglevel.INFO.value)
        self.log("homing", Loglevel.INFO.value)

        # StallGuard only works with StealthChop
        self.set_spreadcycle(0)

        self.log("Stallguard threshold:"+str(self._sg_threshold), Loglevel.DEBUG.value)

        self.set_stallguard_callback(diag_pin, self._sg_threshold, self.stop,
                                     0.5*tmc_math.rps_to_steps(speed_rpm/60, self._steps_per_rev))

        homing_failed = self.set_vactual_rpm(speed_rpm, revolutions=revolutions)

        if homing_failed:
            self.log("homing failed", Loglevel.INFO.value)
        else:
            self.log("homing successful",Loglevel.INFO.value)

        self._current_pos = 0

        self.log("---", Loglevel.INFO.value)
        return not homing_failed



    def do_homing2(self, revolutions, threshold=None):
        """
        homes the motor in the given direction using stallguard
        old function, uses STEP/DIR 

            Parameters
                revolutions (int): max number of revolutions. Can be negative for inverse direction
                threshold (int): optional; StallGuard detection threshold
        """
        sg_results = []

        if threshold is not None:
            self._sg_threshold = threshold

        self.log("---", Loglevel.INFO.value)
        self.log("homing", Loglevel.INFO.value)

        self.log("Stallguard threshold:"+str(self._sg_threshold), Loglevel.DEBUG.value)

        self.set_direction_pin(revolutions > 0)

        # StallGuard only works with StealthChop
        self.set_spreadcycle(0)

        self._target_pos = self._steps_per_rev * revolutions
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self.set_acceleration(10000)
        self.set_max_speed(self._max_speed_homing)
        coolstep_thres = tmc_math.steps_to_tstep(self._max_speed_homing*0.5, self._msres)
        self.set_coolstep_threshold(coolstep_thres)
        self.compute_new_speed()


        step_counter=0
        #self.log("Steps per Revolution: "+str(self._steps_per_rev))
        while step_counter<self._target_pos:
            if self.run_speed(): #returns true, when a step is made
                step_counter += 1
                self.compute_new_speed()
                sg_result = self.get_stallguard_result()
                sg_results.append(sg_result)
                if len(sg_results)>20:
                    sg_result_average = statistics.mean(sg_results[-6:])
                    if sg_result_average < self._sg_threshold:
                        break

        if step_counter<self._steps_per_rev:
            self.log("homing successful",Loglevel.INFO.value)
            self.log("Stepcounter: "+str(step_counter),Loglevel.DEBUG.value)
            self.log(str(sg_results),Loglevel.DEBUG.value)
            self._current_pos = 0
        else:
            self.log("homing failed", Loglevel.INFO.value)
            self.log("Stepcounter: "+str(step_counter), Loglevel.DEBUG.value)
            self.log(str(sg_results),Loglevel.DEBUG.value)

        self.log("---", Loglevel.INFO.value)



    def get_current_position(self):
        """
        returns the current motor position in microsteps
        """
        return self._current_pos



    def set_current_position(self, new_pos):
        """
        overwrites the current motor position in microsteps
        """
        self._current_pos = new_pos



    def reverse_direction_pin(self):
        """
        reverses the motor shaft direction
        """
        self._direction = not self._direction
        GPIO.output(self._pin_dir, self._direction)



    def set_direction_pin(self, direction):
        """
        sets the motor shaft direction to the given value: 0 = CCW; 1 = CW
        """
        self._direction = direction
        GPIO.output(self._pin_dir, direction)



    def get_direction_reg(self):
        """
        returns the motor shaft direction: 0 = CCW; 1 = CW
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        return gconf & tmc_reg.shaft



    def set_direction_reg(self, direction):
        """
        sets the motor shaft direction to the given value: 0 = CCW; 1 = CW
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        if direction:
            self.log("write inverse motor direction", Loglevel.INFO.value)
            gconf = self.tmc_uart.set_bit(gconf, tmc_reg.shaft)
        else:
            self.log("write normal motor direction", Loglevel.INFO.value)
            gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.shaft)
        self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



    def get_iscale_analog(self):
        """
        return whether Vref (1) or 5V (0) is used for current scale
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        return gconf & tmc_reg.i_scale_analog



    def set_iscale_analog(self,en):
        """
        sets Vref (1) or 5V (0) for current scale
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        if en:
            self.log("activated Vref for current scale", Loglevel.INFO.value)
            gconf = self.tmc_uart.set_bit(gconf, tmc_reg.i_scale_analog)
        else:
            self.log("activated 5V-out for current scale", Loglevel.INFO.value)
            gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.i_scale_analog)
        self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



    def getvsense(self):
        """
        returns which sense resistor voltage is used for current scaling
        0: Low sensitivity, high sense resistor voltage
        1: High sensitivity, low sense resistor voltage
        """
        chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
        return chopconf & tmc_reg.vsense



    def setvsense(self,en):
        """
        sets which sense resistor voltage is used for current scaling
        0: Low sensitivity, high sense resistor voltage
        1: High sensitivity, low sense resistor voltage
        """
        chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
        if en:
            self.log("activated High sensitivity, low sense resistor voltage", Loglevel.INFO.value)
            chopconf = self.tmc_uart.set_bit(chopconf, tmc_reg.vsense)
        else:
            self.log("activated Low sensitivity, high sense resistor voltage", Loglevel.INFO.value)
            chopconf = self.tmc_uart.clear_bit(chopconf, tmc_reg.vsense)
        self.tmc_uart.write_reg_check(tmc_reg.CHOPCONF, chopconf)



    def get_internal_rsense(self):
        """
        returns which sense resistor voltage is used for current scaling
        0: Low sensitivity, high sense resistor voltage
        1: High sensitivity, low sense resistor voltage
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        return gconf & tmc_reg.internal_rsense



    def set_internal_rsense(self,en):
        """
        sets which sense resistor voltage is used for current scaling
        0: Low sensitivity, high sense resistor voltage
        1: High sensitivity, low sense resistor voltage
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        if en:
            self.log("activated internal sense resistors.", Loglevel.INFO.value)
            self.log("VREF pin internally is driven to GND in this mode.", Loglevel.INFO.value)
            self.log("This will most likely destroy your driver!!!", Loglevel.INFO.value)
            raise SystemExit
            # gconf = self.tmc_uart.set_bit(gconf, tmc_reg.internal_rsense)
        self.log("activated operation with external sense resistors", Loglevel.INFO.value)
        gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.internal_rsense)
        self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



    def set_irun_ihold(self, IHold, IRun, ihold_delay):
        """
        sets the current scale (CS) for Running and Holding
        and the delay, when to be switched to Holding current
        IHold = 0-31; IRun = 0-31; ihold_delay = 0-15
        """
        ihold_irun = 0

        ihold_irun = ihold_irun | IHold << 0
        ihold_irun = ihold_irun | IRun << 8
        ihold_irun = ihold_irun | ihold_delay << 16
        self.log("ihold_irun", Loglevel.INFO.value)
        self.log(str(bin(ihold_irun)), Loglevel.INFO.value)

        self.log("writing ihold_irun", Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(tmc_reg.IHOLD_IRUN, ihold_irun)



    def set_pdn_disable(self,pdn_disable):
        """
        disables PDN on the UART pin
        0: PDN_UART controls standstill current reduction
        1: PDN_UART input function disabled. Set this bit,
        when using the UART interface!
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        if pdn_disable:
            self.log("enabled PDN_UART", Loglevel.INFO.value)
            gconf = self.tmc_uart.set_bit(gconf, tmc_reg.pdn_disable)
        else:
            self.log("disabled PDN_UART", Loglevel.INFO.value)
            gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.pdn_disable)
        self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



    def set_current(self, run_current, hold_current_multiplier = 0.5, hold_current_delay = 10,
                    use_vref=False, Vref = 1.2, pdn_disable = True):
        """
        sets the current flow for the motor
        run_current in mA
        check whether Vref is actually 1.2V
        """
        cs_irun = 0
        rsense = 0.11
        vfs = 0

        if use_vref:
            self.set_iscale_analog(True)
        else:
            Vref=2.5
            self.set_iscale_analog(False)

        vfs = 0.325 * Vref / 2.5
        cs_irun = 32.0*1.41421*run_current/1000.0*(rsense+0.02)/vfs - 1

        # If Current Scale is too low, turn on high sensitivity VSsense and calculate again
        if cs_irun < 16:
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



    def get_spreadcycle(self):
        """
        return whether spreadcycle (1) is active or stealthchop (0)
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        return gconf & tmc_reg.en_spreadcycle



    def set_spreadcycle(self,en_spread):
        """
        enables spreadcycle (1) or stealthchop (0)
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
        if en_spread:
            self.log("activated Spreadcycle", Loglevel.INFO.value)
            gconf = self.tmc_uart.set_bit(gconf, tmc_reg.en_spreadcycle)
        else:
            self.log("activated Stealthchop", Loglevel.INFO.value)
            gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.en_spreadcycle)
        self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



    def get_interpolation(self):
        """
        return whether the tmc inbuilt interpolation is active
        """
        chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
        return bool(chopconf & tmc_reg.intpol)



    def set_interpolation(self, en):
        """
        enables the tmc inbuilt interpolation of the steps to 256 microsteps
        """
        chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)

        if en:
            chopconf = self.tmc_uart.set_bit(chopconf, tmc_reg.intpol)
        else:
            chopconf = self.tmc_uart.clear_bit(chopconf, tmc_reg.intpol)

        self.log("writing microstep interpolation setting: "+str(en), Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(tmc_reg.CHOPCONF, chopconf)



    def get_microstepping_resolution(self):
        """
        returns the current native microstep resolution (1-256)
        """
        chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)

        msresdezimal = chopconf & (tmc_reg.msres0 | tmc_reg.msres1 |
                                   tmc_reg.msres2 | tmc_reg.msres3)

        msresdezimal = msresdezimal >> 24
        msresdezimal = 8 - msresdezimal

        self._msres = int(math.pow(2, msresdezimal))

        return self._msres



    def set_microstepping_resolution(self, msres):
        """
        sets the current native microstep resolution (1,2,4,8,16,32,64,128,256)
        """
        chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
        #setting all bits to zero
        chopconf = chopconf & (~tmc_reg.msres0 | ~tmc_reg.msres1 |
                               ~tmc_reg.msres2 | ~tmc_reg.msres3)
        msresdezimal = int(math.log(msres, 2))
        msresdezimal = 8 - msresdezimal
        chopconf = int(chopconf) & int(4043309055)
        chopconf = chopconf | msresdezimal <<24

        self.log("writing "+str(msres)+" microstep setting", Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(tmc_reg.CHOPCONF, chopconf)

        self.set_mstep_resolution_reg_select(True)

        self.read_steps_per_rev()

        return True



    def set_mstep_resolution_reg_select(self, en):
        """
        sets the register bit "mstep_reg_select" to 1 or 0 depending to the given value.
        this is needed to set the microstep resolution via UART
        this method is called by "set_microstepping_resolution"
        """
        gconf = self.tmc_uart.read_int(tmc_reg.GCONF)

        if en is True:
            gconf = self.tmc_uart.set_bit(gconf, tmc_reg.mstep_reg_select)
        else:
            gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.mstep_reg_select)

        self.log("writing MStep Reg Select: "+str(en), Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



    def read_steps_per_rev(self):
        """
        returns how many steps are needed for one revolution
        """
        self._steps_per_rev = self._fullsteps_per_rev*self.get_microstepping_resolution()
        return self._steps_per_rev



    def get_steps_per_rev(self):
        """
        returns how many steps are needed for one revolution
        """
        return self._steps_per_rev



    def get_interface_transmission_counter(self):
        """
        reads the interface transmission counter from the tmc register
        this value is increased on every succesfull write access
        can be used to verify a write access

            Returns:
                ifcnt (int): 8bit IFCNT Register
        """
        ifcnt = self.tmc_uart.read_int(tmc_reg.IFCNT)
        self.log("Interface Transmission Counter: "+str(ifcnt), Loglevel.INFO.value)
        return ifcnt



    def get_tstep(self):
        """
        reads the current tstep from the driver register
        """
        tstep = self.tmc_uart.read_int(tmc_reg.TSTEP)
        return tstep



    def set_vactual(self, vactual, duration=0, acceleration=0, show_stallguard_result=False,
                    show_tstep=False):
        """
        sets the register bit "VACTUAL" to to a given value
        VACTUAL allows moving the motor by UART control.
        It gives the motor velocity in +-(2^23)-1 [μsteps / t]
        0: Normal operation. Driver reacts to STEP input
        """
        self._stop = StopMode.NO
        current_vactual = 0
        sleeptime = 0.05
        if vactual<0:
            acceleration = -acceleration

        if duration != 0:
            self.log("vactual: "+str(vactual)+" for "+str(duration)+" sec", Loglevel.INFO.value)
        else:
            self.log("vactual: "+str(vactual), Loglevel.INFO.value)
        self.log(str(bin(vactual)), Loglevel.INFO.value)

        self.log("writing vactual", Loglevel.INFO.value)
        if acceleration == 0:
            self.tmc_uart.write_reg_check(tmc_reg.VACTUAL, vactual)

        if duration == 0:
            return -1

        self._starttime = time.time()
        current_time = time.time()
        while current_time < self._starttime+duration:
            if self._stop == StopMode.HARDSTOP:
                break
            if acceleration != 0:
                time_to_stop = self._starttime+duration-abs(current_vactual/acceleration)
                if self._stop == StopMode.SOFTSTOP:
                    time_to_stop = current_time-1
            if acceleration != 0 and current_time > time_to_stop:
                current_vactual -= acceleration*sleeptime
                self.tmc_uart.write_reg_check(tmc_reg.VACTUAL, int(round(current_vactual)))
                time.sleep(sleeptime)
            elif acceleration != 0 and abs(current_vactual)<abs(vactual):
                current_vactual += acceleration*sleeptime
                self.tmc_uart.write_reg_check(tmc_reg.VACTUAL, int(round(current_vactual)))
                time.sleep(sleeptime)
            if show_stallguard_result:
                self.log("StallGuard result: "+str(self.get_stallguard_result()),
                            Loglevel.INFO.value)
                time.sleep(0.1)
            if show_tstep:
                self.log("TStep result: "+str(self.get_tstep()), Loglevel.INFO.value)
                time.sleep(0.1)
            current_time = time.time()
        self.tmc_uart.write_reg_check(tmc_reg.VACTUAL, 0)
        return self._stop



    def set_vactual_rps(self, rps, duration=0, revolutions=0, acceleration=0):
        """
        converts the rps parameter to a vactual value which represents
        rotation speed in revolutions per second
        With internal oscillator:
        VACTUAL[2209] = v[Hz] / 0.715Hz
        """
        if revolutions == 0:
            return -1
        print("vactual")
        vactual = tmc_math.rps_to_vactual(rps, self._steps_per_rev)
        duration = abs(revolutions/rps)
        if revolutions<0:
            vactual = -vactual
        return self.set_vactual(vactual, duration, acceleration=acceleration)



    def set_vactual_rpm(self, rpm, duration=0, revolutions=0, acceleration=0):
        """
        converts the rps parameter to a vactual value which represents
        rotation speed in revolutions per minute
        """
        return self.set_vactual_rps(rpm/60, duration, revolutions, acceleration)



    def get_stallguard_result(self):
        """
        return the current stallguard result
        its will be calculated with every fullstep
        higher values means a lower motor load

            Returns:
                sg_result (int): StallGuard Result
        """
        sg_result = self.tmc_uart.read_int(tmc_reg.SG_RESULT)
        return sg_result



    def set_stallguard_threshold(self, threshold):
        """
        sets the register bit "SGTHRS" to to a given value
        this is needed for the stallguard interrupt callback
        SG_RESULT becomes compared to the double of this threshold.
        SG_RESULT ≤ SGTHRS*2

            Parameters:
                threshold (int): value for SGTHRS
        """

        self.log("sgthrs", Loglevel.INFO.value)
        self.log(str(bin(threshold)), Loglevel.INFO.value)

        self.log("writing sgthrs", Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(tmc_reg.SGTHRS, threshold)



    def set_coolstep_threshold(self, threshold):
        """
        This  is  the  lower  threshold  velocity  for  switching  
        on  smart energy CoolStep and StallGuard to DIAG output. (unsigned)
        """

        self.log("tcoolthrs", Loglevel.INFO.value)
        self.log(str(bin(threshold)), Loglevel.INFO.value)

        self.log("writing tcoolthrs", Loglevel.INFO.value)
        self.tmc_uart.write_reg_check(tmc_reg.TCOOLTHRS, threshold)



    def set_stallguard_callback(self, pin_stallguard, threshold, callback,
                                min_speed = 100, ignore_delay = 0):
        """
        set a function to call back, when the driver detects a stall 
        via stallguard
        high value on the diag pin can also mean a driver error

            Parameters:
                pin_stallguard (int): pin needs to be connected to DIAG
                threshold (int): value for SGTHRS
                callback (function): will be called on StallGuard trigger
                min_speed (int): min speed [steps/s] for StallGuard
        """
        self.log("setup stallguard callback on GPIO"+str(pin_stallguard), Loglevel.INFO.value)
        self.log("StallGuard Threshold: "+str(threshold)+"\tminimum Speed: "+str(min_speed),
                 Loglevel.INFO.value)

        self.set_stallguard_threshold(threshold)
        self.set_coolstep_threshold(tmc_math.steps_to_tstep(min_speed, self._msres))
        self._sg_delay = ignore_delay
        self._sg_callback = callback
        self._pin_stallguard = pin_stallguard

        GPIO.setup(self._pin_stallguard, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        GPIO.add_event_detect(self._pin_stallguard, GPIO.RISING, callback=self.stallguard_callback,
                              bouncetime=300)



    def stallguard_callback(self, channel):
        """
        the callback function for StallGuard.
        only checks whether the duration of the current movement is longer than
        _sg_delay and then calls the actual callback
        """
        if self._sg_callback is None:
            self.log("StallGuard callback is None", Loglevel.DEBUG.value)
            return
        if time.time()<=self._starttime+self._sg_delay and self._sg_delay != 0:
            return
        self._sg_callback(channel)



    def get_microstep_counter(self):
        """
        returns the current Microstep counter.
        Indicates actual position in the microstep table for CUR_A
        """
        mscnt = self.tmc_uart.read_int(tmc_reg.MSCNT)
        return mscnt



    def get_microstep_counter_in_steps(self, offset=0):
        """
        returns the current Microstep counter.
        Indicates actual position in the microstep table for CUR_A
        """
        step = (self.get_microstep_counter()-64)*(self._msres*4)/1024
        step = (4*self._msres)-step-1
        step = round(step)
        return step+offset



    def set_max_speed(self, speed):
        """
        sets the maximum motor speed in µsteps per second
        """
        if speed < 0.0:
            speed = -speed
        if self._max_speed != speed:
            self._max_speed = speed
            self._cmin = 1000000.0 / speed
            # Recompute _n from current speed and adjust speed if accelerating or cruising
            if self._n > 0:
                self._n = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
                self.compute_new_speed()



    def set_max_speed_fullstep(self, speed):
        """
        sets the maximum motor speed in fullsteps per second
        """
        self.set_max_speed(speed*self._msres)



    def get_max_speed(self):
        """
        returns the maximum motor speed in steps per second
        """
        return self._max_speed



    def set_acceleration(self, acceleration):
        """
        sets the motor acceleration/decceleration in µsteps per sec per sec
        """
        if acceleration == 0.0:
            return
        acceleration = abs(acceleration)
        if self._acceleration != acceleration:
            # Recompute _n per Equation 17
            self._n = self._n * (self._acceleration / acceleration)
            # New c0 per Equation 7, with correction per Equation 15
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
            self._acceleration = acceleration
            self.compute_new_speed()



    def set_acceleration_fullstep(self, acceleration):
        """
        sets the motor acceleration/decceleration in fullsteps per sec per sec
        """
        self.set_acceleration(acceleration*self._msres)



    def get_acceleration(self):
        """
        returns the motor acceleration/decceleration in steps per sec per sec
        """
        return self._acceleration



    def stop(self, stop_mode = StopMode.HARDSTOP):
        """
        stop the current movement
        """
        self._stop = stop_mode



    def get_movement_phase(self):
        """
        return the current Movement Phase
        """
        return self._movement_phase



    def run_to_position_steps(self, steps, movement_abs_rel = None):
        """
        runs the motor to the given position.
        with acceleration and deceleration
        blocks the code until finished or stopped from a different thread!
        returns true when the movement if finshed normally and false,
        when the movement was stopped
        """
        if movement_abs_rel is None:
            movement_abs_rel = self._movement_abs_rel

        if movement_abs_rel == MovementAbsRel.RELATIVE:
            self._target_pos = self._current_pos + steps
        else:
            self._target_pos = steps

        self._stop = StopMode.NO
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self.compute_new_speed()
        while self.run(): #returns false, when target position is reached
            if self._stop == StopMode.HARDSTOP:
                break

        self._movement_phase = MovementPhase.STANDSTILL
        return self._stop



    def run_to_position_revolutions(self, revolutions, movement_absolute_relative = None):
        """
        runs the motor to the given position.
        with acceleration and deceleration
        blocks the code until finished!
        """
        return self.run_to_position_steps(round(revolutions * self._steps_per_rev),
                                          movement_absolute_relative)



    def run_to_position_steps_threaded(self, steps, movement_abs_rel = None):
        """
        runs the motor to the given position.
        with acceleration and deceleration
        does not block the code
        returns true when the movement if finshed normally and false,
        when the movement was stopped
        """
        self._movement_thread = threading.Thread(target=self.run_to_position_steps,
                                                 args=(steps, movement_abs_rel))
        self._movement_thread.start()



    def run_to_position_revolutions_threaded(self, revolutions, movement_absolute_relative = None):
        """
        runs the motor to the given position.
        with acceleration and deceleration
        does not block the code
        """
        return self.run_to_position_steps_threaded(round(revolutions * self._steps_per_rev),
                                                   movement_absolute_relative)



    def wait_for_movement_finished_threaded(self):
        """
        wait for the motor to finish the movement,
        if startet threaded
        returns true when the movement if finshed normally and false,
        when the movement was stopped
        """
        self._movement_thread.join()
        return self._stop



    def run(self):
        """
        calculates a new speed if a speed was made
        returns true if the target position is reached
        should not be called from outside!
        """
        if self.run_speed(): #returns true, when a step is made
            self.compute_new_speed()
            #self.log(self.get_stallguard_result())
            #self.log(self.get_tstep())
        return (self._speed != 0.0 and self.distance_to_go() != 0)



    def distance_to_go(self):
        """
        returns the remaining distance the motor should run
        """
        return self._target_pos - self._current_pos



    def compute_new_speed(self):
        """
        returns the calculated current speed depending on the acceleration
        this code is based on: 
        "Generate stepper-motor speed profiles in real time" by David Austin

        https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
        https://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
        """
        distance_to = self.distance_to_go() # +ve is clockwise from curent location
        steps_to_stop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
        if ((distance_to == 0 and steps_to_stop <= 2) or
        (self._stop == StopMode.SOFTSTOP and steps_to_stop <= 1)):
            # We are at the target and its time to stop
            self._step_interval = 0
            self._speed = 0.0
            self._n = 0
            self._movement_phase = MovementPhase.STANDSTILL
            self.log("time to stop", Loglevel.MOVEMENT.value)
            return

        if distance_to > 0:
            # We are anticlockwise from the target
            # Need to go clockwise from here, maybe decelerate now
            if self._n > 0:
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((steps_to_stop >= distance_to) or self._direction == Direction.CCW or
                    self._stop == StopMode.SOFTSTOP):
                    self._n = -steps_to_stop # Start deceleration
                    self._movement_phase = MovementPhase.DECELERATING
            elif self._n < 0:
                # Currently decelerating, need to accel again?
                if (steps_to_stop < distance_to) and self._direction == Direction.CW:
                    self._n = -self._n # Start accceleration
                    self._movement_phase = MovementPhase.ACCELERATING
        elif distance_to < 0:
            # We are clockwise from the target
            # Need to go anticlockwise from here, maybe decelerate
            if self._n > 0:
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if (((steps_to_stop >= -distance_to) or self._direction == Direction.CW or
                    self._stop == StopMode.SOFTSTOP)):
                    self._n = -steps_to_stop # Start deceleration
                    self._movement_phase = MovementPhase.DECELERATING
            elif self._n < 0:
                # Currently decelerating, need to accel again?
                if (steps_to_stop < -distance_to) and self._direction == Direction.CCW:
                    self._n = -self._n # Start accceleration
                    self._movement_phase = MovementPhase.ACCELERATING
        # Need to accelerate or decelerate
        if self._n == 0:
            # First step from stopped
            self._cn = self._c0
            GPIO.output(self._pin_step, GPIO.LOW)
            #self.log("distance to: " + str(distance_to))
            if distance_to > 0:
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
            if self._cn == self._cmin:
                self._movement_phase = MovementPhase.MAXSPEED
        self._n += 1
        self._step_interval = self._cn
        self._speed = 1000000.0 / self._cn
        if self._direction == 0:
            self._speed = -self._speed



    def run_speed(self):
        """
        this methods does the actual steps with the current speed
        """
        # Dont do anything unless we actually have a step interval
        if not self._step_interval:
            return False

        curtime = time.time_ns()/1000

        #self.log("current time: " + str(curtime))
        #self.log("last st time: " + str(self._last_step_time))

        if curtime - self._last_step_time >= self._step_interval:

            if self._direction == 1: # Clockwise
                self._current_pos += 1
            else: # Anticlockwise
                self._current_pos -= 1
            self.make_a_step()

            self._last_step_time = curtime # Caution: does not account for costs in step()
            return True
        return False



    def make_a_step(self):
        """
        method that makes on step
        for the TMC2209 there needs to be a signal duration of minimum 100 ns
        """
        GPIO.output(self._pin_step, GPIO.HIGH)
        time.sleep(1/1000/1000)
        GPIO.output(self._pin_step, GPIO.LOW)
        time.sleep(1/1000/1000)

        self.log("one step", Loglevel.MOVEMENT.value)



    def test_dir_step_en(self):
        """
        tests the EN, DIR and STEP pin
        this sets the EN, DIR and STEP pin to HIGH, LOW and HIGH
        and checks the IOIN Register of the TMC meanwhile
        """
        pin_dir_ok = pin_step_ok = pin_en_ok = True

        GPIO.output(self._pin_step, GPIO.HIGH)
        GPIO.output(self._pin_dir, GPIO.HIGH)
        GPIO.output(self._pin_en, GPIO.HIGH)
        time.sleep(0.1)
        ioin = self.readIOIN()
        if not ioin & tmc_reg.io_dir:
            pin_dir_ok = False
        if not ioin & tmc_reg.io_step:
            pin_step_ok = False
        if not ioin & tmc_reg.io_enn:
            pin_en_ok = False

        GPIO.output(self._pin_step, GPIO.LOW)
        GPIO.output(self._pin_dir, GPIO.LOW)
        GPIO.output(self._pin_en, GPIO.LOW)
        time.sleep(0.1)
        ioin = self.readIOIN()
        if ioin & tmc_reg.io_dir:
            pin_dir_ok = False
        if ioin & tmc_reg.io_step:
            pin_step_ok = False
        if ioin & tmc_reg.io_enn:
            pin_en_ok = False

        GPIO.output(self._pin_step, GPIO.HIGH)
        GPIO.output(self._pin_dir, GPIO.HIGH)
        GPIO.output(self._pin_en, GPIO.HIGH)
        time.sleep(0.1)
        ioin = self.readIOIN()
        if not ioin & tmc_reg.io_dir:
            pin_dir_ok = False
        if not ioin & tmc_reg.io_step:
            pin_step_ok = False
        if not ioin & tmc_reg.io_enn:
            pin_en_ok = False

        self.set_motor_enabled(False)

        self.log("---")
        if pin_dir_ok:
            self.log("Pin DIR: \tOK")
        else:
            self.log("Pin DIR: \tnot OK")
        if pin_step_ok:
            self.log("Pin STEP: \tOK")
        else:
            self.log("Pin STEP: \tnot OK")
        if pin_en_ok:
            self.log("Pin EN: \tOK")
        else:
            self.log("Pin EN: \tnot OK")
        self.log("---")



    def test_step(self):
        """
        test method
        """
        self.set_direction_pin(1)

        for _ in range(100):
            self._current_pos += 1
            GPIO.output(self._pin_step, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(self._pin_step, GPIO.LOW)
            time.sleep(0.01)



    def test_uart(self):
        """
        test method
        """
        self.log("---")
        self.log("TEST UART")
        result = self.tmc_uart.test_uart(tmc_reg.IOIN)

        snd = result[0]
        rtn = result[1]

        self.log("length snd: "+str(len(snd)), Loglevel.DEBUG.value)
        self.log("length rtn: "+str(len(rtn)), Loglevel.DEBUG.value)

        if len(rtn)==12:
            self.log("the Raspberry Pi received the sended bits and the answer from the TMC",
                     Loglevel.INFO.value)
        elif len(rtn)==4:
            self.log("the Raspberry Pi received only the sended bits", Loglevel.INFO.value)
        elif len(rtn)==0:
            self.log("the Raspberry Pi did not receive anything", Loglevel.INFO.value)
        else:
            self.log("the Raspberry Pi received an unexpected amount of bits: "+
                     str(len(rtn)), Loglevel.INFO.value)

        if snd[0:4] == rtn[0:4]:
            self.log("""the Raspberry Pi received exactly the bits it has send.
                     the first 4 bits are the same""", Loglevel.INFO.value)
        else:
            self.log("""the Raspberry Pi did not received the bits it has send.
                     the first 4 bits are different""", Loglevel.INFO.value)


        self.log("complete", Loglevel.DEBUG.value)
        self.log(str(snd.hex()), Loglevel.DEBUG.value)
        self.log(str(rtn.hex()), Loglevel.DEBUG.value)

        self.log("just the first 4 bits", Loglevel.DEBUG.value)
        self.log(str(snd[0:4].hex()), Loglevel.DEBUG.value)
        self.log(str(rtn[0:4].hex()), Loglevel.DEBUG.value)


        self.log("---")
        return True



    def test_stallguard_threshold(self, steps):
        """
        test method for tuning stallguard threshold
        run this function with your motor settings and your motor load
        the function will determine the minimum stallguard results for each movement phase
        """

        self.log("---", Loglevel.INFO.value)
        self.log("test_stallguard_threshold", Loglevel.INFO.value)

        self.set_spreadcycle(0)

        min_stallguard_result_accel = 511
        min_stallguard_result_maxspeed = 511
        min_stallguard_result_decel = 511

        self.run_to_position_steps_threaded(steps, MovementAbsRel.RELATIVE)


        while self._movement_phase != MovementPhase.STANDSTILL:
            stallguard_result = self.get_stallguard_result()

            self.log(str(self._movement_phase) + " | " + str(stallguard_result),
                     Loglevel.INFO.value)

            if (self._movement_phase == MovementPhase.ACCELERATING and
               stallguard_result < min_stallguard_result_accel):
                min_stallguard_result_accel = stallguard_result
            if (self._movement_phase == MovementPhase.MAXSPEED and
               stallguard_result < min_stallguard_result_maxspeed):
                min_stallguard_result_maxspeed = stallguard_result
            if (self._movement_phase == MovementPhase.DECELERATING and
               stallguard_result < min_stallguard_result_decel):
                min_stallguard_result_decel = stallguard_result

        self.wait_for_movement_finished_threaded()

        self.log("---", Loglevel.INFO.value)
        self.log("min StallGuard result during acceleration: " +
                 str(min_stallguard_result_accel), Loglevel.INFO.value)
        self.log("min StallGuard result during maxspeed: " +
                 str(min_stallguard_result_maxspeed), Loglevel.INFO.value)
        self.log("min StallGuard result during deceleration: " +
                 str(min_stallguard_result_decel), Loglevel.INFO.value)
        self.log("---", Loglevel.INFO.value)
