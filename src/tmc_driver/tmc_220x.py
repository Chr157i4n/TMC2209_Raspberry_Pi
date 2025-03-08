#pylint: disable=too-many-arguments
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=too-many-instance-attributes
#pylint: disable=too-many-positional-arguments
#pylint: disable=bare-except
#pylint: disable=no-member
#pylint: disable=unused-import
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""Tmc220X stepper driver module

this module has two different functions:
1. change setting in the TMC-driver via UART
2. move the motor via STEP/DIR pins
"""

import logging
import threading
import time
import typing
from ._tmc_stepperdriver import *
from .com._tmc_com import TmcCom
from .com._tmc_com_uart import TmcComUart
from .com._tmc_com_spi import TmcComSpi
from .motion_control._tmc_mc_step_reg import TmcMotionControlStepReg
from .enable_control._tmc_ec_toff import TmcEnableControlToff
from .motion_control._tmc_mc_vactual import TmcMotionControlVActual
from ._tmc_logger import TmcLogger, Loglevel
from .reg._tmc220x_reg import *
from . import _tmc_math as tmc_math





class Tmc220x(TmcStepperDriver):
    """Tmc220X

    this class has two different functions:
    1. change setting in the TMC-driver via UART
    2. move the motor via STEP/DIR pins
    """

    tmc_com:TmcComUart = None


# Constructor/Destructor
# ----------------------------
    def __init__(self,
                    tmc_ec:TmcEnableControl,
                    tmc_mc:TmcMotionControl,
                    tmc_com:TmcCom = None,
                    driver_address:int = 0,
                    gpio_mode = None,
                    loglevel:Loglevel = Loglevel.INFO,
                    logprefix:str = None,
                    log_handlers:list = None,
                    log_formatter:logging.Formatter = None
                    ):
        """constructor

        Args:
            tmc_ec (TmcEnableControl): enable control object
            tmc_mc (TmcMotionControl): motion control object
            tmc_com (TmcCom, optional): communication object. Defaults to None.
            driver_address (int, optional): driver address [0-3]. Defaults to 0.
            gpio_mode (enum, optional): gpio mode. Defaults to None.
            loglevel (enum, optional): loglevel. Defaults to None.
            logprefix (str, optional): log prefix (name of the logger).
                Defaults to None (standard TMC prefix).
            log_handlers (list, optional): list of logging handlers.
                Defaults to None (log to console).
            log_formatter (logging.Formatter, optional): formatter for the log messages.
                Defaults to None (messages are logged in the format
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s').
        """
        super().__init__(tmc_ec, tmc_mc, gpio_mode, loglevel, logprefix, log_handlers, log_formatter)

        self.tmc_logger.set_logprefix(f"TMC2209 {driver_address}")

        if tmc_com is not None:
            self.tmc_com = tmc_com
            self.tmc_com.tmc_logger = self.tmc_logger
            self.tmc_com.mtr_id = driver_address

            self.tmc_com.init()

            if hasattr(self.tmc_mc, "tmc_com"):
                self.tmc_mc.tmc_com = tmc_com

            if hasattr(self.tmc_ec, "tmc_com"):
                self.tmc_ec.tmc_com = tmc_com

            registers_classes = {
                GConf,
                GStat,
                IfCnt,
                Ioin,
                IHoldIRun,
                TPowerDown,
                TStep,
                VActual,
                MsCnt,
                ChopConf,
                DrvStatus
            }

            self.tmc_registers = {}

            for register_class in registers_classes:
                register = register_class(self.tmc_com)
                name = register.name.lower()
                self.tmc_registers[name] = register

                def create_getter(name):
                    def getter(self):
                        return self.tmc_registers[name]
                    return getter

                setattr(self.__class__, name, property(create_getter(name)))


        if tmc_com is not None:
            # Setup Registers
            self.tmc_com.tmc_registers = self.tmc_registers

            if self.tmc_mc is not None:
                self.read_steps_per_rev()
            self.clear_gstat()
            self.tmc_com.flush_serial_buffer()


        self.max_speed_fullstep = 100
        self.acceleration_fullstep = 100

        self.tmc_logger.log("TMC220x Init finished", Loglevel.INFO)



    def __del__(self):
        """destructor"""
        if self.tmc_com is not None:
            del self.tmc_com
        super().__del__()



    def set_deinitialize_true(self):
        """set deinitialize to true"""
        self._deinit_finished = True


# Tmc220x methods
# ----------------------------
    def read_steps_per_rev(self) -> int:
        """returns how many steps are needed for one revolution.
        this reads the value from the tmc driver.

        Returns:
            int: Steps per revolution
        """
        self.read_microstepping_resolution()
        return self.tmc_mc.steps_per_rev



    def read_drv_status(self) -> DrvStatus:
        """read the register Adress "DRV_STATUS" and logs the reg valuess

        Returns:
            DRV_STATUS Register instance
        """
        self.drvstatus.read()
        self.drvstatus.log(self.tmc_logger)
        return self.drvstatus



    def read_gconf(self) -> GConf:
        """read the register Adress "GCONF" and logs the reg values

        Returns:
            GCONF Register instance
        """
        self.gconf.read()
        self.gconf.log(self.tmc_logger)
        return self.gconf



    def read_gstat(self) -> GStat:
        """read the register Adress "GSTAT" and logs the reg values

        Returns:
            GSTAT Register instance
        """
        self.gstat.read()
        self.gstat.log(self.tmc_logger)
        return self.gstat



    def clear_gstat(self):
        """clears the "GSTAT" register"""
        self.tmc_logger.log("clearing GSTAT", Loglevel.INFO)
        self.gstat.read()

        self.gstat.reset = True
        self.gstat.drv_err = True
        self.gstat.uv_cp = True

        self.gstat.write_check()



    def read_ioin(self) -> Ioin:
        """read the register Adress "IOIN" and logs the reg values

        Returns:
            IOIN Register instance
        """
        self.ioin.read()
        self.ioin.log(self.tmc_logger)
        return self.ioin



    def read_chopconf(self) -> ChopConf:
        """read the register Adress "CHOPCONF" and logs the reg values

        Returns:
            CHOPCONF Register instance
        """
        self.chopconf.read()
        self.chopconf.log(self.tmc_logger)
        return self.chopconf



    def get_direction_reg(self) -> bool:
        """returns the motor shaft direction: False = CCW; True = CW

        Returns:
            bool: motor shaft direction: False = CCW; True = CW
        """
        self.gconf.read()
        return self.gconf.shaft



    def set_direction_reg(self, direction:bool):
        """sets the motor shaft direction to the given value: False = CCW; True = CW

        Args:
            direction (bool): direction of the motor False = CCW; True = CW
        """
        self.gconf.modify("shaft", direction)



    def get_iscale_analog(self) -> bool:
        """return whether Vref (True) or 5V (False) is used for current scale

        Returns:
            en (bool): whether Vref (True) or 5V (False) is used for current scale
        """
        self.gconf.read()
        return self.gconf.i_scale_analog



    def set_iscale_analog(self,en:bool):
        """sets Vref (True) or 5V (False) for current scale

        Args:
            en (bool): True=Vref, False=5V
        """
        self.gconf.modify("i_scale_analog", en)



    def get_vsense(self) -> bool:
        """returns which sense resistor voltage is used for current scaling
        False: Low sensitivity, high sense resistor voltage
        True: High sensitivity, low sense resistor voltage

        Returns:
            bool: whether high sensitivity should is used
        """
        self.chopconf.read()
        return self.chopconf.vsense



    def set_vsense(self,en:bool):
        """sets which sense resistor voltage is used for current scaling
        False: Low sensitivity, high sense resistor voltage
        True: High sensitivity, low sense resistor voltage

        Args:
            en (bool):
        """
        self.chopconf.modify("vsense", en)



    def get_internal_rsense(self) -> bool:
        """returns which sense resistor voltage is used for current scaling
        False: Operation with external sense resistors
        True Internal sense resistors. Use current supplied into
        VREF as reference for internal sense resistor. VREF
        pin internally is driven to GND in this mode.

        Returns:
            bool: which sense resistor voltage is used
        """
        self.gconf.read()
        return self.gconf.internal_rsense



    def set_internal_rsense(self,en:bool):
        """sets which sense resistor voltage is used for current scaling
        False: Operation with external sense resistors
        True: Internal sense resistors. Use current supplied into
        VREF as reference for internal sense resistor. VREF
        pin internally is driven to GND in this mode.

        Args:
        en (bool): which sense resistor voltage is used; true will propably destroy your tmc

            """
        if en:
            self.tmc_logger.log("activated internal sense resistors.",
                                Loglevel.INFO)
            self.tmc_logger.log("VREF pin internally is driven to GND in this mode.",
                                Loglevel.INFO)
            self.tmc_logger.log("This will most likely destroy your driver!!!",
                                Loglevel.INFO)
            raise SystemExit


        self.gconf.modify("internal_rsense", en)



    def _set_irun_ihold(self, ihold:int, irun:int, ihold_delay:int):
        """sets the current scale (CS) for Running and Holding
        and the delay, when to be switched to Holding current

        Args:
        ihold (int): multiplicator for current while standstill [0-31]
        irun (int): current while running [0-31]
        ihold_delay (int): delay after standstill for switching to ihold [0-15]

        """
        self.ihold_irun.read()

        self.ihold_irun.ihold = ihold
        self.ihold_irun.irun = irun
        self.ihold_irun.ihold_delay = ihold_delay

        self.ihold_irun.write_check()



    def _set_pdn_disable(self,pdn_disable:bool):
        """disables PDN on the UART pin
        False: PDN_UART controls standstill current reduction
        True: PDN_UART input function disabled. Set this bit,
        when using the UART interface!

        Args:
            pdn_disable (bool): whether PDN should be disabled
        """
        self.gconf.modify("pdn_disable", pdn_disable)



    def set_current(self, run_current:int, hold_current_multiplier:float = 0.5,
                    hold_current_delay:int = 10, pdn_disable:bool = True):
        """sets the current flow for the motor.

        Args:
        run_current (int): current during movement in mA
        hold_current_multiplier (int):current multiplier during standstill (Default value = 0.5)
        hold_current_delay (int): delay after standstill after which cur drops (Default value = 10)
        pdn_disable (bool): should be disabled if UART is used (Default value = True)

        """
        cs_irun = 0
        rsense = 0.11
        vfs = 0

        self.set_iscale_analog(False)

        vfs = 0.325
        cs_irun = 32.0*1.41421*run_current/1000.0*(rsense+0.02)/vfs - 1

        # If Current Scale is too low, turn on high sensitivity VSsense and calculate again
        if cs_irun < 16:
            self.tmc_logger.log("CS too low; switching to VSense True", Loglevel.INFO)
            vfs = 0.180
            cs_irun = 32.0*1.41421*run_current/1000.0*(rsense+0.02)/vfs - 1
            self.set_vsense(True)
        else: # If CS >= 16, turn off high_senser
            self.tmc_logger.log("CS in range; using VSense False", Loglevel.INFO)
            self.set_vsense(False)

        cs_irun = min(cs_irun, 31)
        cs_irun = max(cs_irun, 0)

        cs_ihold = hold_current_multiplier * cs_irun

        cs_irun = round(cs_irun)
        cs_ihold = round(cs_ihold)
        hold_current_delay = round(hold_current_delay)

        self.tmc_logger.log(f"cs_irun: {cs_irun}", Loglevel.INFO)
        self.tmc_logger.log(f"CS_IHold: {cs_ihold}", Loglevel.INFO)
        self.tmc_logger.log(f"Delay: {hold_current_delay}", Loglevel.INFO)

        # return (float)(CS+1)/32.0 * (vsense() ? 0.180 : 0.325)/(rsense+0.02) / 1.41421 * 1000;
        run_current_actual = (cs_irun+1)/32.0 * (vfs)/(rsense+0.02) / 1.41421 * 1000
        self.tmc_logger.log(f"actual current: {round(run_current_actual)} mA",
                            Loglevel.INFO)

        self._set_irun_ihold(cs_ihold, cs_irun, hold_current_delay)

        self._set_pdn_disable(pdn_disable)



    def get_spreadcycle(self) -> bool:
        """reads spreadcycle

        Returns:
            bool: True = spreadcycle; False = stealthchop
        """
        self.gconf.read()
        return self.gconf.spreadcycle



    def set_spreadcycle(self,en:bool):
        """enables spreadcycle (1) or stealthchop (0)

        Args:
        en (bool): true to enable spreadcycle; false to enable stealthchop

        """
        self.gconf.modify("spreadcycle", en)



    def get_interpolation(self) -> bool:
        """return whether the tmc inbuilt interpolation is active

        Returns:
            en (bool): true if internal µstep interpolation is enabled
        """
        self.chopconf.read()
        return self.chopconf.intpol



    def set_interpolation(self, en:bool):
        """enables the tmc inbuilt interpolation of the steps to 256 µsteps

        Args:
            en (bool): true to enable internal µstep interpolation
        """
        self.chopconf.modify("intpol", en)



    def get_toff(self) -> int:
        """returns the TOFF register value

        Returns:
            int: TOFF register value
        """
        self.chopconf.read()
        return self.chopconf.toff



    def set_toff(self, toff:int):
        """Sets TOFF register to value

        Args:
            toff (uint8_t): value of toff (must be a four-bit value)
        """
        self.chopconf.modify("toff", toff)



    def read_microstepping_resolution(self) -> int:
        """returns the current native microstep resolution (1-256)
        this reads the value from the driver register

        Returns:
            int: µstep resolution
        """
        self.chopconf.read()

        mres = self.chopconf.mres_ms
        if self.tmc_mc is not None:
            self.tmc_mc.mres = mres

        return mres



    def get_microstepping_resolution(self) -> int:
        """returns the current native microstep resolution (1-256)
        this returns the cached value from this module

        Returns:
            int: µstep resolution
        """
        return self.tmc_mc.mres



    def set_microstepping_resolution(self, mres:int):
        """sets the current native microstep resolution (1,2,4,8,16,32,64,128,256)

        Args:
            mres (int): µstep resolution; has to be a power of 2 or 1 for fullstep
        """
        if self.tmc_mc is not None:
            self.tmc_mc.mres = mres

        self.chopconf.read()
        self.chopconf.mres_ms = mres
        self.chopconf.write_check()

        self.set_mstep_resolution_reg_select(True)



    def set_mstep_resolution_reg_select(self, en:bool):
        """sets the register bit "mstep_reg_select" to 1 or 0 depending to the given value.
        this is needed to set the microstep resolution via UART
        this method is called by "set_microstepping_resolution"

        Args:
            en (bool): true to set µstep resolution via UART
        """
        self.gconf.modify("mstep_reg_select", en)



    def get_interface_transmission_counter(self) -> int:
        """reads the interface transmission counter from the tmc register
        this value is increased on every succesfull write access
        can be used to verify a write access

        Returns:
            int: 8bit IFCNT Register
        """
        self.ifcnt.read()
        ifcnt = self.ifcnt.ifcnt
        self.tmc_logger.log(f"Interface Transmission Counter: {ifcnt}", Loglevel.INFO)
        return ifcnt



    def get_tstep(self) -> int:
        """reads the current tstep from the driver register

        Returns:
            int: TStep time
        """
        self.chopconf.read()
        return self.chopconf.tstep



    def set_vactual(self, vactual:int):
        """sets the register bit "VACTUAL" to to a given value
        VACTUAL allows moving the motor by UART control.
        It gives the motor velocity in +-(2^23)-1 [μsteps / t]
        0: Normal operation. Driver reacts to STEP input

        Args:
            vactual (int): value for VACTUAL
        """
        self.vactual.vactual = vactual
        self.vactual.write_check()



    def get_microstep_counter(self) -> int:
        """returns the current Microstep counter.
        Indicates actual position in the microstep table for CUR_A

        Returns:
            int: current Microstep counter
        """
        self.mscnt.read()
        return self.mscnt.mscnt



    def get_microstep_counter_in_steps(self, offset:int=0) -> int:
        """returns the current Microstep counter.
        Indicates actual position in the microstep table for CUR_A

        Args:
            offset (int): offset in steps (Default value = 0)

        Returns:
            step (int): current Microstep counter convertet to steps
        """
        step = (self.get_microstep_counter()-64)*(self.tmc_mc.mres*4)/1024
        step = (4*self.tmc_mc.mres)-step-1
        step = round(step)
        return step+offset



    def test_pin(self, pin, ioin_reg_bp):
        """tests one pin

        this function checks the connection to a pin
        by toggling it and reading the IOIN register
        """
        pin_ok = True

        # turn on all pins
        tmc_gpio.gpio_output(self.tmc_mc.pin_dir, Gpio.HIGH)
        tmc_gpio.gpio_output(self.tmc_mc.pin_step, Gpio.HIGH)
        tmc_gpio.gpio_output(self.tmc_ec.pin_en, Gpio.HIGH)

        # check that the selected pin is on
        ioin = self.read_ioin()
        if not ioin.data >> ioin_reg_bp & 0x1:
            pin_ok = False

        # turn off only the selected pin
        tmc_gpio.gpio_output(pin, Gpio.LOW)
        time.sleep(0.1)

        # check that the selected pin is off
        ioin = self.read_ioin()
        if ioin.data >> ioin_reg_bp & 0x1:
            pin_ok = False

        return pin_ok



    def test_dir_step_en(self):
        """tests the EN, DIR and STEP pin

        this sets the EN, DIR and STEP pin to HIGH, LOW and HIGH
        and checks the IOIN Register of the TMC meanwhile
        """
        # test each pin on their own
        pin_dir_ok = self.test_pin(self.tmc_mc.pin_dir, 9)
        pin_step_ok = self.test_pin(self.tmc_mc.pin_step, 7)
        pin_en_ok = self.test_pin(self.tmc_ec.pin_en, 0)

        self.set_motor_enabled(False)

        self.tmc_logger.log("---")
        self.tmc_logger.log(f"Pin DIR: \t{'OK' if pin_dir_ok else 'not OK'}")
        self.tmc_logger.log(f"Pin STEP: \t{'OK' if pin_step_ok else 'not OK'}")
        self.tmc_logger.log(f"Pin EN: \t{'OK' if pin_en_ok else 'not OK'}")
        self.tmc_logger.log("---")



    def test_com(self):
        """test method"""
        self.tmc_logger.log("---")
        self.tmc_logger.log("TEST COM")

        ioin = Ioin(self.tmc_com)

        return self.tmc_com.test_com(ioin.addr)
