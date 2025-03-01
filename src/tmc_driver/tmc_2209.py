#pylint: disable=too-many-arguments
#pylint: disable=too-many-instance-attributes
#pylint: disable=import-outside-toplevel
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""Tmc2209 stepper driver module
"""

import statistics
import types
from .tmc_220x import *
from ._tmc_gpio_board import GpioPUD



class Tmc2209(Tmc220x):
    """Tmc2209"""

    _pin_stallguard:int = None
    _sg_callback:types.FunctionType = None
    _sg_threshold:int = 100             # threshold for stallguard



    from ._tmc_test import (
        test_stallguard_threshold
    )


    def __del__(self):
        """destructor"""
        if self._deinit_finished is False:
            if self._pin_stallguard is not None:
                tmc_gpio.gpio_remove_event_detect(self._pin_stallguard)
                tmc_gpio.gpio_cleanup(self._pin_stallguard)

        super().__del__()


    def set_stallguard_callback(self, pin_stallguard, threshold, callback,
                                min_speed = 100):
        """set a function to call back, when the driver detects a stall
        via stallguard
        high value on the diag pin can also mean a driver error

        Args:
            pin_stallguard (int): pin needs to be connected to DIAG
            threshold (int): value for SGTHRS
            callback (func): will be called on StallGuard trigger
            min_speed (int): min speed [steps/s] for StallGuard (Default value = 100)
        """
        self.tmc_logger.log(f"setup stallguard callback on GPIO {pin_stallguard}",
                            Loglevel.INFO)
        self.tmc_logger.log(f"""StallGuard Threshold: {threshold}
                            minimum Speed: {min_speed}""", Loglevel.INFO)

        self.set_stallguard_threshold(threshold)
        self.set_coolstep_threshold(tmc_math.steps_to_tstep(
            min_speed, self.get_microstepping_resolution()))
        self._sg_callback = callback
        self._pin_stallguard = pin_stallguard

        tmc_gpio.gpio_setup(self._pin_stallguard, GpioMode.IN, pull_up_down=GpioPUD.PUD_DOWN)
        # first remove existing events
        tmc_gpio.gpio_remove_event_detect(self._pin_stallguard)
        tmc_gpio.gpio_add_event_detect(self._pin_stallguard, self.stallguard_callback)



    def stallguard_callback(self, gpio_pin):
        """the callback function for StallGuard.
        only checks whether the duration of the current movement is longer than
        _sg_delay and then calls the actual callback

        Args:
            gpio_pin (int): pin number of the interrupt pin
        """
        del gpio_pin
        if self._sg_callback is None:
            self.tmc_logger.log("StallGuard callback is None", Loglevel.DEBUG)
            return
        self._sg_callback()



    def do_homing(self, diag_pin, revolutions = 10, threshold = None, speed_rpm = None):
        """homes the motor in the given direction using stallguard.
        this method is using vactual to move the motor and an interrupt on the DIAG pin

        Args:
            diag_pin (int): DIAG pin number
            revolutions (int): max number of revolutions. Can be negative for inverse direction
                (Default value = 10)
            threshold (int): StallGuard detection threshold (Default value = None)
            speed_rpm (float):speed in revolutions per minute (Default value = None)

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

        self.tmc_logger.log(f"Stallguard threshold: {self._sg_threshold}", Loglevel.DEBUG)

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
        """homes the motor in the given direction using stallguard
        old function, uses STEP/DIR to move the motor and pulls the StallGuard result
        from the interface

        Args:
            revolutions (int): max number of revolutions. Can be negative for inverse direction
            threshold (int, optional): StallGuard detection threshold (Default value = None)
        """
        sg_results = []

        if threshold is not None:
            self._sg_threshold = threshold

        self.tmc_logger.log("---", Loglevel.INFO)
        self.tmc_logger.log("homing", Loglevel.INFO)

        self.tmc_logger.log(f"Stallguard threshold: {self._sg_threshold}", Loglevel.DEBUG)

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
        #self.tmc_logger.log("Steps per Revolution: {self._steps_per_rev}"")
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
            self.tmc_logger.log(f"Stepcounter: {step_counter}",Loglevel.DEBUG)
            self.tmc_logger.log(str(sg_results),Loglevel.DEBUG)
            self._current_pos = 0
        else:
            self.tmc_logger.log("homing failed", Loglevel.INFO)
            self.tmc_logger.log(f"Stepcounter: {step_counter}", Loglevel.DEBUG)
            self.tmc_logger.log(str(sg_results),Loglevel.DEBUG)

        self.tmc_logger.log("---", Loglevel.INFO)
