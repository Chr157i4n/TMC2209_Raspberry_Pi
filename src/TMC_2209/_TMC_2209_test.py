#pylint: disable=invalid-name
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=protected-access
#pylint: disable=no-member
#pylint: disable=bare-except
"""
TMC_2209 stepper driver communication module
"""

import time
from ._TMC_2209_GPIO_board import TMC_gpio, Gpio
from ._TMC_2209_logger import Loglevel
from ._TMC_2209_move import MovementAbsRel, MovementPhase
from . import _TMC_2209_reg as tmc_reg



def test_dir_step_en(self):
    """tests the EN, DIR and STEP pin

    this sets the EN, DIR and STEP pin to HIGH, LOW and HIGH
    and checks the IOIN Register of the TMC meanwhile
    """
    pin_dir_ok = pin_step_ok = pin_en_ok = True

    TMC_gpio.gpio_output(self._pin_dir, Gpio.HIGH)
    TMC_gpio.gpio_output(self._pin_step, Gpio.HIGH)
    TMC_gpio.gpio_output(self._pin_en, Gpio.HIGH)
    time.sleep(0.1)
    ioin = self.read_ioin()
    if not ioin & tmc_reg.io_dir:
        pin_dir_ok = False
    if not ioin & tmc_reg.io_step:
        pin_step_ok = False
    if not ioin & tmc_reg.io_enn:
        pin_en_ok = False

    TMC_gpio.gpio_output(self._pin_dir, Gpio.HIGH)
    TMC_gpio.gpio_output(self._pin_step, Gpio.LOW)
    TMC_gpio.gpio_output(self._pin_en, Gpio.HIGH)
    time.sleep(0.1)
    ioin = self.read_ioin()
    if not ioin & tmc_reg.io_dir:
        pin_dir_ok = False
    if ioin & tmc_reg.io_step:
        pin_step_ok = False
    if not ioin & tmc_reg.io_enn:
        pin_en_ok = False

    TMC_gpio.gpio_output(self._pin_dir, Gpio.LOW)
    TMC_gpio.gpio_output(self._pin_step, Gpio.HIGH)
    TMC_gpio.gpio_output(self._pin_en, Gpio.HIGH)
    time.sleep(0.1)
    ioin = self.read_ioin()
    if ioin & tmc_reg.io_dir:
        pin_dir_ok = False
    if not ioin & tmc_reg.io_step:
        pin_step_ok = False
    if not ioin & tmc_reg.io_enn:
        pin_en_ok = False

    TMC_gpio.gpio_output(self._pin_dir, Gpio.HIGH)
    TMC_gpio.gpio_output(self._pin_step, Gpio.HIGH)
    TMC_gpio.gpio_output(self._pin_en, Gpio.LOW)
    time.sleep(0.1)
    ioin = self.read_ioin()
    if not ioin & tmc_reg.io_dir:
        pin_dir_ok = False
    if not ioin & tmc_reg.io_step:
        pin_step_ok = False
    if ioin & tmc_reg.io_enn:
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
    """test method"""
    self.set_direction_pin(1)

    for _ in range(100):
        self._current_pos += 1
        TMC_gpio.gpio_output(self._pin_step, Gpio.HIGH)
        time.sleep(0.001)
        TMC_gpio.gpio_output(self._pin_step, Gpio.LOW)
        time.sleep(0.01)



def test_uart(self):
    """test method"""
    self.tmc_logger.log("---")
    self.tmc_logger.log("TEST UART")
    result = self.tmc_uart.test_uart(tmc_reg.IOIN)

    snd = result[0]
    rtn = result[1]

    status = True

    self.tmc_logger.log(f"length snd: {len(snd)}", Loglevel.DEBUG)
    self.tmc_logger.log(f"length rtn: {len(rtn)}", Loglevel.DEBUG)


    self.tmc_logger.log("complete messages:", Loglevel.DEBUG)
    self.tmc_logger.log(str(snd.hex()), Loglevel.DEBUG)
    self.tmc_logger.log(str(rtn.hex()), Loglevel.DEBUG)

    self.tmc_logger.log("just the first 4 bits:", Loglevel.DEBUG)
    self.tmc_logger.log(str(snd[0:4].hex()), Loglevel.DEBUG)
    self.tmc_logger.log(str(rtn[0:4].hex()), Loglevel.DEBUG)

    if len(rtn)==12:
        self.tmc_logger.log("""the Raspberry Pi received the sent
                            bits and the answer from the TMC""", Loglevel.DEBUG)
    elif len(rtn)==4:
        self.tmc_logger.log("the Raspberry Pi received only the sent bits",
                            Loglevel.ERROR)
        status = False
    elif len(rtn)==0:
        self.tmc_logger.log("the Raspberry Pi did not receive anything",
                            Loglevel.ERROR)
        status = False
    else:
        self.tmc_logger.log(f"the Raspberry Pi received an unexpected amount of bits: {len(rtn)}",
                            Loglevel.ERROR)
        status = False

    if snd[0:4] == rtn[0:4]:
        self.tmc_logger.log("""the Raspberry Pi received exactly the bits it has send.
                    the first 4 bits are the same""", Loglevel.DEBUG)
    else:
        self.tmc_logger.log("""the Raspberry Pi did not received the bits it has send.
                    the first 4 bits are different""", Loglevel.DEBUG)
        status = False

    self.tmc_logger.log("---")
    if status:
        self.tmc_logger.log("UART connection: OK", Loglevel.INFO)
    else:
        self.tmc_logger.log("UART connection: not OK", Loglevel.ERROR)

    self.tmc_logger.log("---")
    return True



def test_stallguard_threshold(self, steps):
    """test method for tuning stallguard threshold

    run this function with your motor settings and your motor load
    the function will determine the minimum stallguard results for each movement phase

    Args:
        steps (int):
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

        self.tmc_logger.log(f"{self._movement_phase} | {stallguard_result}",
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
    self.tmc_logger.log(f"min StallGuard result during accel: {min_stallguard_result_accel}",
                        Loglevel.INFO)
    self.tmc_logger.log(f"min StallGuard result during maxspeed: {min_stallguard_result_maxspeed}",
    Loglevel.INFO)
    self.tmc_logger.log(f"min StallGuard result during decel: {min_stallguard_result_decel}",
                        Loglevel.INFO)
    self.tmc_logger.log("---", Loglevel.INFO)
