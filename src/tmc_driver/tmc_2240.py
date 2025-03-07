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
from .reg._tmc224x_reg import *
from . import _tmc_math as tmc_math





class Tmc2240(TmcStepperDriver):
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

        self.tmc_logger.set_logprefix(f"TMC2240 {driver_address}")

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
                GlobalScaler,
                IHoldIRun,
                TPowerDown,
                TStep,
                ADCVSupplyAIN,
                ADCTemp,
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

        self.tmc_logger.log("TMC2240 Init finished", Loglevel.INFO)



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



    def set_irun_ihold(self, ihold:int, irun:int, ihold_delay:int, irun_delay:int):
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
        self.ihold_irun.iholddelay = ihold_delay
        self.ihold_irun.irundelay = irun_delay

        self.ihold_irun.write_check()



    def set_global_scaler(self, scaler:int):
        """sets the global scaler

        Args:
            scaler (int): global scaler value
        """
        self.gscaler.scaler = scaler
        self.gscaler.write_check()



    def set_current(self, run_current:int, hold_current_multiplier:float = 0.5,
                    hold_current_delay:int = 10, run_current_delay:int = 0):
        """sets the current flow for the motor.

        Args:
        run_current (int): current during movement in mA
        hold_current_multiplier (int):current multiplier during standstill (Default value = 0.5)
        hold_current_delay (int): delay after standstill after which cur drops (Default value = 10)
        """
        # TODO: change algorithm for TMC2240
        cs_irun = 0
        rdson = 0.23    # 230 mOhm
        vfs = 0

        # vfs = 0.325
        # cs_irun = 32.0*1.41421*run_current/1000.0*(rsense+0.02)/vfs - 1

        # cs_irun = min(cs_irun, 31)
        # cs_irun = max(cs_irun, 0)

        # cs_ihold = hold_current_multiplier * cs_irun

        # cs_irun = round(cs_irun)
        # cs_ihold = round(cs_ihold)
        # hold_current_delay = round(hold_current_delay)

        # self.tmc_logger.log(f"CS_IRun: {cs_irun}", Loglevel.INFO)
        # self.tmc_logger.log(f"CS_IHold: {cs_ihold}", Loglevel.INFO)
        # self.tmc_logger.log(f"IHold_Delay: {hold_current_delay}", Loglevel.INFO)
        # self.tmc_logger.log(f"IRun_Delay: {run_current_delay}", Loglevel.INFO)

        # run_current_actual = (cs_irun+1)/32.0 * (vfs)/(rsense+0.02) / 1.41421 * 1000
        # self.tmc_logger.log(f"actual current: {round(run_current_actual)} mA",
        #                     Loglevel.INFO)

        # self.set_irun_ihold(cs_ihold, cs_irun, hold_current_delay, run_current_delay)



    def get_spreadcycle(self) -> bool:
        """reads spreadcycle

        Returns:
            bool: True = spreadcycle; False = stealthchop
        """
        self.gconf.read()
        return not self.gconf.en_pwm_mode



    def set_spreadcycle(self,en:bool):
        """enables spreadcycle (1) or stealthchop (0)

        Args:
        en (bool): true to enable spreadcycle; false to enable stealthchop

        """
        self.gconf.modify("en_pwm_mode", not en)



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

        mres = self.chopconf.convert_reg_to_mres()
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
        self.chopconf.convert_mres_to_reg(mres)
        self.chopconf.write_check()



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
        self.tstep.read()
        return self.tstep.tstep



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



    def get_vsupply(self) -> int:
        """reads the ADC_VSUPPLY_AIN register

        Returns:
            int: ADC_VSUPPLY_AIN register value
        """
        self.adcv_supply_ain.read()
        return self.adcv_supply_ain.vsupply



    def get_temperature(self) -> float:
        """reads the ADC_TEMP register and returns the temperature

        Returns:
            float: temperature in °C
        """
        self.adc_temp.read()
        return self.adc_temp.temperature



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

        pin_dir_ok = self.test_pin(self.tmc_mc.pin_dir, 1)
        pin_step_ok = self.test_pin(self.tmc_mc.pin_step, 0)
        pin_en_ok = self.test_pin(self.tmc_ec.pin_en, 4)

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
        result = self.tmc_com.test_com(self.ioin.addr)

        snd = result[0]
        rtn = result[1]

        status = True

        self.tmc_logger.log(f"length snd: {len(snd)}", Loglevel.DEBUG)
        self.tmc_logger.log(f"length rtn: {len(rtn)}", Loglevel.DEBUG)


        self.tmc_logger.log("complete messages:", Loglevel.DEBUG)
        self.tmc_logger.log(str(snd.hex()), Loglevel.DEBUG)
        self.tmc_logger.log(str(rtn.hex()), Loglevel.DEBUG)

        self.tmc_logger.log("just the first 4 bytes:", Loglevel.DEBUG)
        self.tmc_logger.log(str(snd[0:4].hex()), Loglevel.DEBUG)
        self.tmc_logger.log(str(rtn[0:4].hex()), Loglevel.DEBUG)

        if len(rtn)==12:
            self.tmc_logger.log("""the Raspberry Pi received the sent
                                bytes and the answer from the TMC""", Loglevel.DEBUG)
        elif len(rtn)==4:
            self.tmc_logger.log("the Raspberry Pi received only the sent bytes",
                                Loglevel.ERROR)
            status = False
        elif len(rtn)==0:
            self.tmc_logger.log("the Raspberry Pi did not receive anything",
                                Loglevel.ERROR)
            status = False
        else:
            self.tmc_logger.log(f"the Raspberry Pi received an unexpected amount of bytes: {len(rtn)}",
                                Loglevel.ERROR)
            status = False

        if snd[0:4] == rtn[0:4]:
            self.tmc_logger.log("""the Raspberry Pi received exactly the bytes it has send.
                        the first 4 bytes are the same""", Loglevel.DEBUG)
        else:
            self.tmc_logger.log("""the Raspberry Pi did not received the bytes it has send.
                        the first 4 bytes are different""", Loglevel.DEBUG)
            status = False

        self.tmc_logger.log("---")
        if status:
            self.tmc_logger.log("UART connection: OK", Loglevel.INFO)
        else:
            self.tmc_logger.log("UART connection: not OK", Loglevel.ERROR)

        self.tmc_logger.log("---")
        return status
