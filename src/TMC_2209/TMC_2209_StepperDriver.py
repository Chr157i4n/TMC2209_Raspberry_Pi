#pylint: disable=invalid-name
#pylint: disable=import-error
#pylint: disable=too-many-arguments
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=too-many-instance-attributes
#pylint: disable=import-outside-toplevel
"""
TMC_2209 stepper driver module
"""

import time
import statistics
from RPi import GPIO
from ._TMC_2209_uart import TMC_UART as tmc_uart
from ._TMC_2209_logger import TMC_logger, Loglevel
from ._TMC_2209_move import MovementAbsRel, MovementPhase, StopMode
from . import _TMC_2209_reg as tmc_reg
from . import _TMC_2209_math as tmc_math




class TMC_2209:
    """
    TMC_2209

    this class has two different functions:
    1. change setting in the TMC-driver via UART
    2. move the motor via STEP/DIR pins
    """

    from ._TMC_2209_comm import (
        read_drv_status, read_gconf, read_gstat, clear_gstat, read_ioin, read_chopconf,
        get_direction_reg, set_direction_reg, get_iscale_analog, set_iscale_analog,get_vsense,
        set_vsense, get_internal_rsense, set_internal_rsense, set_irun_ihold, set_pdn_disable,
        set_current, get_spreadcycle, set_spreadcycle, get_interpolation, set_interpolation,
        read_microstepping_resolution, get_microstepping_resolution, set_microstepping_resolution,
        set_mstep_resolution_reg_select, get_interface_transmission_counter, get_tstep, set_vactual,
        get_stallguard_result, set_stallguard_threshold, set_coolstep_threshold,
        get_microstep_counter, get_microstep_counter_in_steps
    )

    from ._TMC_2209_move import (
        set_max_speed, set_max_speed_fullstep, get_max_speed, set_acceleration,
        set_acceleration_fullstep, get_acceleration, stop, get_movement_phase,
        run_to_position_steps, run_to_position_revolutions, run_to_position_steps_threaded,
        run_to_position_revolutions_threaded, wait_for_movement_finished_threaded,
        run, distance_to_go, compute_new_speed, run_speed, make_a_step
    )

    tmc_uart = None
    tmc_logger = None
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
                 driver_address=0, gpio_mode=GPIO.BCM, loglevel=None, skip_uart_init=False):
        """
        constructor
        """
        self.init(pin_en, pin_step, pin_dir, baudrate, serialport, driver_address,
                  gpio_mode, loglevel, skip_uart_init)



    def __del__(self):
        """
        destructor
        """
        self.deinit()
        del self.tmc_uart
        del self.tmc_logger



    def init(self, pin_en, pin_step=-1, pin_dir=-1, baudrate=115200, serialport="/dev/serial0",
    driver_address=0, gpio_mode=GPIO.BCM, loglevel=None, skip_uart_init=False):
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
        self.tmc_logger = TMC_logger(loglevel, "TMC2209 "+str(driver_address))
        self.tmc_uart = tmc_uart(self.tmc_logger, serialport, baudrate, driver_address)


        self.tmc_logger.log("Init", Loglevel.INFO)
        GPIO.setwarnings(False)
        GPIO.setmode(gpio_mode)

        self.tmc_logger.log("EN Pin: " + str(pin_en), Loglevel.DEBUG)
        self._pin_en = pin_en
        GPIO.setup(self._pin_en, GPIO.OUT, initial=GPIO.HIGH)

        self.tmc_logger.log("STEP Pin: " + str(pin_step), Loglevel.DEBUG)
        if pin_step != -1:
            self._pin_step = pin_step
            GPIO.setup(self._pin_step, GPIO.OUT, initial=GPIO.LOW)

        self.tmc_logger.log("DIR Pin: " + str(pin_dir), Loglevel.DEBUG)
        if pin_dir != -1:
            self._pin_dir = pin_dir
            GPIO.setup(self._pin_dir, GPIO.OUT, initial=self._direction)

        self.tmc_logger.log("GPIO Init finished", Loglevel.INFO)

        if not skip_uart_init:
            self.read_steps_per_rev()
            self.clear_gstat()

        self.tmc_uart.flush_serial_buffer()
        self.tmc_logger.log("Init finished", Loglevel.INFO)



    def deinit(self):
        """
        deinit function
        """
        if self._deinit_finished is False:
            self.tmc_logger.log("Deinit", Loglevel.INFO)

            self.set_motor_enabled(False)

            self.tmc_logger.log("GPIO cleanup")
            if self._pin_step != -1:
                GPIO.cleanup(self._pin_step)
            if self._pin_dir != -1:
                GPIO.cleanup(self._pin_dir)
            if self._pin_en != -1:
                GPIO.cleanup(self._pin_en)
            if self._pin_stallguard != -1:
                GPIO.remove_event_detect(self._pin_stallguard)
                GPIO.cleanup(self._pin_stallguard)

            self.tmc_logger.log("Deinit finished", Loglevel.INFO)
            self._deinit_finished= True
        else:
            self.tmc_logger.log("Deinit already finished", Loglevel.INFO)



    def set_deinitialize_true(self):
        """
        set deinitialize to true
        """
        self._deinit_finished = True



    def set_movement_abs_rel(self, movement_abs_rel):
        """
        set whether the movment should be relative or absolute by default.
        See the Enum MovementAbsoluteRelative

            Paramters:
                movement_abs_rel (enum): whether the movment should be relative or absolute
        """
        self._movement_abs_rel = movement_abs_rel



    def set_motor_enabled(self, en):
        """
        enables or disables the motor current output

            Parameters:
                en (bool): whether the motor current output should be enabled
        """
        GPIO.output(self._pin_en, not en)
        self.tmc_logger.log(f"Motor output active: {en}", Loglevel.INFO)



    def do_homing(self, diag_pin, revolutions = 10, threshold = None, speed_rpm = None):
        """
        homes the motor in the given direction using stallguard.
        this method is using vactual to move the motor and an interrupt on the DIAG pin

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

        self.tmc_logger.log("---", Loglevel.INFO)
        self.tmc_logger.log("homing", Loglevel.INFO)

        # StallGuard only works with StealthChop
        self.set_spreadcycle(0)

        self.tmc_logger.log("Stallguard threshold:"+str(self._sg_threshold), Loglevel.DEBUG)

        self.set_stallguard_callback(diag_pin, self._sg_threshold, self.stop,
                                     0.5*tmc_math.rps_to_steps(speed_rpm/60, self._steps_per_rev))

        homing_failed = self.set_vactual_rpm(speed_rpm, revolutions=revolutions)

        if homing_failed:
            self.tmc_logger.log("homing failed", Loglevel.INFO)
        else:
            self.tmc_logger.log("homing successful",Loglevel.INFO)

        self._current_pos = 0

        self.tmc_logger.log("---", Loglevel.INFO)
        return not homing_failed



    def do_homing2(self, revolutions, threshold=None):
        """
        homes the motor in the given direction using stallguard
        old function, uses STEP/DIR to move the motor and pulls the StallGuard result
        from the interface

            Parameters
                revolutions (int): max number of revolutions. Can be negative for inverse direction
                threshold (int): optional; StallGuard detection threshold
        """
        sg_results = []

        if threshold is not None:
            self._sg_threshold = threshold

        self.tmc_logger.log("---", Loglevel.INFO)
        self.tmc_logger.log("homing", Loglevel.INFO)

        self.tmc_logger.log("Stallguard threshold:"+str(self._sg_threshold), Loglevel.DEBUG)

        self.set_direction_pin(revolutions > 0)

        # StallGuard only works with StealthChop
        self.set_spreadcycle(0)

        self._target_pos = self._steps_per_rev * revolutions
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self.set_acceleration(10000)
        self.set_max_speed(self._max_speed_homing)
        coolstep_thres = tmc_math.steps_to_tstep(self._max_speed_homing*0.5,
                                                 self.get_microstepping_resolution())
        self.set_coolstep_threshold(coolstep_thres)
        self.compute_new_speed()


        step_counter=0
        #self.tmc_logger.log("Steps per Revolution: "+str(self._steps_per_rev))
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
            self.tmc_logger.log("homing successful",Loglevel.INFO)
            self.tmc_logger.log("Stepcounter: "+str(step_counter),Loglevel.DEBUG)
            self.tmc_logger.log(str(sg_results),Loglevel.DEBUG)
            self._current_pos = 0
        else:
            self.tmc_logger.log("homing failed", Loglevel.INFO)
            self.tmc_logger.log("Stepcounter: "+str(step_counter), Loglevel.DEBUG)
            self.tmc_logger.log(str(sg_results),Loglevel.DEBUG)

        self.tmc_logger.log("---", Loglevel.INFO)



    def get_current_position(self):
        """
        returns the current motor position in microsteps

            Returns:
                current_pos (bool): current motor position
        """
        return self._current_pos



    def set_current_position(self, new_pos):
        """
        overwrites the current motor position in microsteps

            Parameters:
                new_pos (bool): new position
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

            Parameters:
                direction (bool): motor shaft direction: 0 = CCW; 1 = CW
        """
        self._direction = direction
        GPIO.output(self._pin_dir, direction)



    def read_steps_per_rev(self):
        """
        returns how many steps are needed for one revolution.
        this reads the value from the tmc driver.

            Returns:
                steps_per_rev (int): Steps per revolution
        """
        self._steps_per_rev = self._fullsteps_per_rev*self.read_microstepping_resolution()
        return self._steps_per_rev



    def get_steps_per_rev(self):
        """
        returns how many steps are needed for one revolution.
        this gets the cached value from the library.

            Returns:
                steps_per_rev (int): Steps per revolution
        """
        return self._steps_per_rev



    def set_vactual_dur(self, vactual, duration=0, acceleration=0,
                             show_stallguard_result=False, show_tstep=False):
        """
        sets the register bit "VACTUAL" to to a given value
        VACTUAL allows moving the motor by UART control.
        It gives the motor velocity in +-(2^23)-1 [μsteps / t]
        0: Normal operation. Driver reacts to STEP input

            Parameters:
                vactual (int): value for vactual
                duration (int): after this vactual will be set to 0
                acceleration (int): use this for a velocity ramp
                show_stallguard_result (bool): prints StallGuard Result during movement
                show_tstep (bool): prints TStep during movement

            Returns:
                stop (enum): how the movement was finished
        """
        self._stop = StopMode.NO
        current_vactual = 0
        sleeptime = 0.05
        if vactual<0:
            acceleration = -acceleration

        if duration != 0:
            self.tmc_logger.log("vactual: "+str(vactual)+" for "+str(duration)+" sec",
                                Loglevel.INFO)
        else:
            self.tmc_logger.log("vactual: "+str(vactual), Loglevel.INFO)
        self.tmc_logger.log(str(bin(vactual)), Loglevel.INFO)

        self.tmc_logger.log("writing vactual", Loglevel.INFO)
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
                self.tmc_uart.write_reg_check(tmc_reg.VACTUAL,
                                                       int(round(current_vactual)))
                time.sleep(sleeptime)
            elif acceleration != 0 and abs(current_vactual)<abs(vactual):
                current_vactual += acceleration*sleeptime
                self.tmc_uart.write_reg_check(tmc_reg.VACTUAL,
                                                       int(round(current_vactual)))
                time.sleep(sleeptime)
            if show_stallguard_result:
                self.tmc_logger.log("StallGuard result: "+
                                    str(self.get_stallguard_result()), Loglevel.INFO)
                time.sleep(0.1)
            if show_tstep:
                self.tmc_logger.log("TStep result: "+str(self.get_tstep()),
                                    Loglevel.INFO)
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

            Parameters:
                rps (int): value for vactual in rps
                duration (int): after this vactual will be set to 0
                revolutions (int): after this vactual will be set to 0
                acceleration (int): use this for a velocity ramp

            Returns:
                stop (enum): how the movement was finished
        """
        vactual = tmc_math.rps_to_vactual(rps, self._steps_per_rev)
        if revolutions!=0:
            duration = abs(revolutions/rps)
        if revolutions<0:
            vactual = -vactual
        return self.set_vactual_dur(vactual, duration, acceleration=acceleration)



    def set_vactual_rpm(self, rpm, duration=0, revolutions=0, acceleration=0):
        """
        converts the rps parameter to a vactual value which represents
        rotation speed in revolutions per minute

            Parameters:
                rpm (int): value for vactual in rpm
                duration (int): after this vactual will be set to 0
                revolutions (int): after this vactual will be set to 0
                acceleration (int): use this for a velocity ramp

            Returns:
                stop (enum): how the movement was finished
        """
        return self.set_vactual_rps(rpm/60, duration, revolutions, acceleration)



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
        self.tmc_logger.log("setup stallguard callback on GPIO"+str(pin_stallguard),
                            Loglevel.INFO)
        self.tmc_logger.log("StallGuard Threshold: "+str(threshold)+"\tminimum Speed: "+
                            str(min_speed), Loglevel.INFO)

        self.set_stallguard_threshold(threshold)
        self.set_coolstep_threshold(tmc_math.steps_to_tstep(
            min_speed, self.get_microstepping_resolution()))
        self._sg_delay = ignore_delay
        self._sg_callback = callback
        self._pin_stallguard = pin_stallguard

        GPIO.setup(self._pin_stallguard, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        GPIO.add_event_detect(self._pin_stallguard, GPIO.RISING, callback=self.stallguard_callback,
                              bouncetime=300)



    def stallguard_callback(self, gpio_pin):
        """
        the callback function for StallGuard.
        only checks whether the duration of the current movement is longer than
        _sg_delay and then calls the actual callback

            Parameters:
                gpio_pin (int): pin number of the interrupt pin
        """
        del gpio_pin
        if self._sg_callback is None:
            self.tmc_logger.log("StallGuard callback is None", Loglevel.DEBUG)
            return
        if time.time()<=self._starttime+self._sg_delay and self._sg_delay != 0:
            return
        self._sg_callback()



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
        ioin = self.read_ioin()
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
        ioin = self.read_ioin()
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
        ioin = self.read_ioin()
        if not ioin & tmc_reg.io_dir:
            pin_dir_ok = False
        if not ioin & tmc_reg.io_step:
            pin_step_ok = False
        if not ioin & tmc_reg.io_enn:
            pin_en_ok = False

        self.set_motor_enabled(False)

        self.tmc_logger.log("---")
        if pin_dir_ok:
            self.tmc_logger.log("Pin DIR: \tOK")
        else:
            self.tmc_logger.log("Pin DIR: \tnot OK")
        if pin_step_ok:
            self.tmc_logger.log("Pin STEP: \tOK")
        else:
            self.tmc_logger.log("Pin STEP: \tnot OK")
        if pin_en_ok:
            self.tmc_logger.log("Pin EN: \tOK")
        else:
            self.tmc_logger.log("Pin EN: \tnot OK")
        self.tmc_logger.log("---")



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
        self.tmc_logger.log("---")
        self.tmc_logger.log("TEST UART")
        result = self.tmc_uart.test_uart(tmc_reg.IOIN)

        snd = result[0]
        rtn = result[1]

        self.tmc_logger.log("length snd: "+str(len(snd)), Loglevel.DEBUG)
        self.tmc_logger.log("length rtn: "+str(len(rtn)), Loglevel.DEBUG)

        if len(rtn)==12:
            self.tmc_logger.log("""the Raspberry Pi received the sended
                                bits and the answer from the TMC""",Loglevel.INFO)
        elif len(rtn)==4:
            self.tmc_logger.log("the Raspberry Pi received only the sended bits",
                                Loglevel.INFO)
        elif len(rtn)==0:
            self.tmc_logger.log("the Raspberry Pi did not receive anything",
                                Loglevel.INFO)
        else:
            self.tmc_logger.log("the Raspberry Pi received an unexpected amount of bits: "+
                     str(len(rtn)), Loglevel.INFO)

        if snd[0:4] == rtn[0:4]:
            self.tmc_logger.log("""the Raspberry Pi received exactly the bits it has send.
                     the first 4 bits are the same""", Loglevel.INFO)
        else:
            self.tmc_logger.log("""the Raspberry Pi did not received the bits it has send.
                     the first 4 bits are different""", Loglevel.INFO)


        self.tmc_logger.log("complete", Loglevel.DEBUG)
        self.tmc_logger.log(str(snd.hex()), Loglevel.DEBUG)
        self.tmc_logger.log(str(rtn.hex()), Loglevel.DEBUG)

        self.tmc_logger.log("just the first 4 bits", Loglevel.DEBUG)
        self.tmc_logger.log(str(snd[0:4].hex()), Loglevel.DEBUG)
        self.tmc_logger.log(str(rtn[0:4].hex()), Loglevel.DEBUG)


        self.tmc_logger.log("---")
        return True



    def test_stallguard_threshold(self, steps):
        """
        test method for tuning stallguard threshold
        run this function with your motor settings and your motor load
        the function will determine the minimum stallguard results for each movement phase

            Parameters:
                steps (int): amount of steps; can be negative
        """

        self.tmc_logger.log("---", Loglevel.INFO)
        self.tmc_logger.log("test_stallguard_threshold", Loglevel.INFO)

        self.set_spreadcycle(0)

        min_stallguard_result_accel = 511
        min_stallguard_result_maxspeed = 511
        min_stallguard_result_decel = 511

        self.run_to_position_steps_threaded(steps, MovementAbsRel.RELATIVE)


        while self._movement_phase != MovementPhase.STANDSTILL:
            stallguard_result = self.get_stallguard_result()

            self.tmc_logger.log(str(self._movement_phase) + " | " + str(stallguard_result),
                     Loglevel.INFO)

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

        self.tmc_logger.log("---", Loglevel.INFO)
        self.tmc_logger.log("min StallGuard result during acceleration: " +
                 str(min_stallguard_result_accel), Loglevel.INFO)
        self.tmc_logger.log("min StallGuard result during maxspeed: " +
                 str(min_stallguard_result_maxspeed), Loglevel.INFO)
        self.tmc_logger.log("min StallGuard result during deceleration: " +
                 str(min_stallguard_result_decel), Loglevel.INFO)
        self.tmc_logger.log("---", Loglevel.INFO)
