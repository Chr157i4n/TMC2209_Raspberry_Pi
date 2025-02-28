#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=protected-access
"""
TMC_2209 stepper driver communication module
"""

from ._tmc_logger import Loglevel
from .reg._tmc_220x_reg_addr import TmcRegAddr
from .reg._tmc_drvstatus import DrvStatus
from .reg._tmc_gconf import GConf
from .reg._tmc_ioin import IOIN
from .reg._tmc_gstat import GStat
from .reg._tmc_ihold_irun import IHoldIRun
from .reg._tmc_chopconf import ChopConf


reg_class_mapping = {
    TmcRegAddr.DRVSTATUS: DrvStatus,
    TmcRegAddr.GCONF: GConf,
    TmcRegAddr.GSTAT: GStat,
    TmcRegAddr.IOIN: IOIN,
    TmcRegAddr.CHOPCONF: ChopConf,
    TmcRegAddr.IHOLD_IRUN: IHoldIRun,
}


def read_reg(self, reg_addr: TmcRegAddr, log:bool = True):
    """read the register Adress and logs the reg valuess

    Returns:
        Register instance
    """
    reg_value =self.tmc_uart.read_int(reg_addr)

    if log:
        self.tmc_logger.log("---", Loglevel.INFO)
        self.tmc_logger.log(f"{reg_addr.name}:", Loglevel.INFO)
        self.tmc_logger.log(bin(reg_value), Loglevel.INFO)

    reg_class = reg_class_mapping[reg_addr]
    reg_instance = reg_class(reg_value)

    if log:
        reg_instance.log( self.tmc_logger )

    return reg_instance



def read_drv_status(self) -> DrvStatus:
    """read the register Adress "DRV_STATUS" and logs the reg valuess

    Returns:
        DRV_STATUS Register instance
    """
    return self.read_reg(TmcRegAddr.DRVSTATUS)



def read_gconf(self) -> GConf:
    """read the register Adress "GCONF" and logs the reg values

    Returns:
        GCONF Register instance
    """
    return self.read_reg(TmcRegAddr.GCONF)



def read_gstat(self) -> GStat:
    """read the register Adress "GSTAT" and logs the reg values

    Returns:
        GSTAT Register instance
    """
    return self.read_reg(TmcRegAddr.GSTAT)



def clear_gstat(self):
    """clears the "GSTAT" register"""
    self.tmc_logger.log("clearing GSTAT", Loglevel.INFO)
    gstat = self.tmc_uart.read_int(TmcRegAddr.GSTAT)

    gstat = GStat(gstat)
    gstat.reset = True
    gstat.drv_err = True
    gstat.uv_cp = True
    gstat_int = gstat.serialise()

    self.tmc_uart.write_reg_check(TmcRegAddr.GSTAT, gstat_int)



def read_ioin(self) -> IOIN:
    """read the register Adress "IOIN" and logs the reg values

    Returns:
        IOIN Register instance
    """
    return self.read_reg(TmcRegAddr.IOIN)



def read_chopconf(self) -> ChopConf:
    """read the register Adress "CHOPCONF" and logs the reg values

    Returns:
        CHOPCONF Register instance
    """
    return self.read_reg(TmcRegAddr.CHOPCONF)



def get_direction_reg(self):
    """returns the motor shaft direction: False = CCW; True = CW

    Returns:
        bool: motor shaft direction: False = CCW; True = CW
    """
    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)

    gconf = GConf(gconf)
    return gconf.shaft



def set_direction_reg(self, direction):
    """sets the motor shaft direction to the given value: False = CCW; True = CW

    Args:
        direction (bool): direction of the motor False = CCW; True = CW
    """
    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)

    gconf = GConf(gconf)
    gconf.shaft = direction
    gconf_int = gconf.serialise()

    self.tmc_uart.write_reg_check(TmcRegAddr.GCONF, gconf_int)
    self._direction = not direction



def get_iscale_analog(self):
    """return whether Vref (True) or 5V (False) is used for current scale

    Returns:
        en (bool): whether Vref (True) or 5V (False) is used for current scale
    """
    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)
    gconf = GConf(gconf)
    return gconf.i_scale_analog



def set_iscale_analog(self,en):
    """sets Vref (True) or 5V (False) for current scale

    Args:
        en (bool): True=Vref, False=5V
    """
    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)

    gconf = GConf(gconf)
    gconf.i_scale_analog = en
    gconf_int = gconf.serialise()

    self.tmc_uart.write_reg_check(TmcRegAddr.GCONF, gconf_int)



def get_vsense(self):
    """returns which sense resistor voltage is used for current scaling
    False: Low sensitivity, high sense resistor voltage
    True: High sensitivity, low sense resistor voltage

    Returns:
        bool: whether high sensitivity should is used
    """
    chopconf = self.tmc_uart.read_int(TmcRegAddr.CHOPCONF)

    chopconf = ChopConf(chopconf)
    return chopconf.vsense



def set_vsense(self,en):
    """sets which sense resistor voltage is used for current scaling
    False: Low sensitivity, high sense resistor voltage
    True: High sensitivity, low sense resistor voltage

    Args:
        en (bool):
    """
    chopconf = self.tmc_uart.read_int(TmcRegAddr.CHOPCONF)

    chopconf = ChopConf(chopconf)
    chopconf.vsense = en
    chopconf_int = chopconf.serialise()

    self.tmc_uart.write_reg_check(TmcRegAddr.CHOPCONF, chopconf_int)



def get_internal_rsense(self):
    """returns which sense resistor voltage is used for current scaling
    False: Operation with external sense resistors
    True Internal sense resistors. Use current supplied into
    VREF as reference for internal sense resistor. VREF
    pin internally is driven to GND in this mode.

    Returns:
        bool: which sense resistor voltage is used
    """
    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)
    gconf = GConf(gconf)
    return gconf.internal_rsense



def set_internal_rsense(self,en):
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


    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)
    gconf = GConf(gconf)
    gconf.internal_rsense = en
    gconf_int = gconf.serialise()

    self.tmc_uart.write_reg_check(TmcRegAddr.GCONF, gconf_int)



def set_irun_ihold(self, ihold, irun, ihold_delay):
    """sets the current scale (CS) for Running and Holding
    and the delay, when to be switched to Holding current

    Args:
      ihold (int): multiplicator for current while standstill [0-31]
      irun (int): current while running [0-31]
      ihold_delay (int): delay after standstill for switching to ihold [0-15]

        """
    ihold_irun = IHoldIRun()
    ihold_irun.ihold = ihold
    ihold_irun.irun = irun
    ihold_irun.iholddelay = ihold_delay
    ihold_irun_int = ihold_irun.serialise()

    self.tmc_uart.write_reg_check(TmcRegAddr.IHOLD_IRUN, ihold_irun_int)



def set_pdn_disable(self,pdn_disable):
    """disables PDN on the UART pin
    False: PDN_UART controls standstill current reduction
    True: PDN_UART input function disabled. Set this bit,
    when using the UART interface!

    Args:
        pdn_disable (bool): whether PDN should be disabled
    """
    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)

    gconf = GConf(gconf)
    gconf.pdn_disable = pdn_disable
    gconf_int = gconf.serialise()

    self.tmc_uart.write_reg_check(TmcRegAddr.GCONF, gconf_int)



def set_current(self, run_current, hold_current_multiplier = 0.5,
                hold_current_delay = 10, pdn_disable = True):
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

    CS_IHold = hold_current_multiplier * cs_irun

    cs_irun = round(cs_irun)
    CS_IHold = round(CS_IHold)
    hold_current_delay = round(hold_current_delay)

    self.tmc_logger.log(f"cs_irun: {cs_irun}", Loglevel.INFO)
    self.tmc_logger.log(f"CS_IHold: {CS_IHold}", Loglevel.INFO)
    self.tmc_logger.log(f"Delay: {hold_current_delay}", Loglevel.INFO)

    # return (float)(CS+1)/32.0 * (vsense() ? 0.180 : 0.325)/(rsense+0.02) / 1.41421 * 1000;
    run_current_actual = (cs_irun+1)/32.0 * (vfs)/(rsense+0.02) / 1.41421 * 1000
    self.tmc_logger.log(f"actual current: {round(run_current_actual)} mA",
                        Loglevel.INFO)

    self.set_irun_ihold(CS_IHold, cs_irun, hold_current_delay)

    self.set_pdn_disable(pdn_disable)



def get_spreadcycle(self):
    """reads spreadcycle

    Returns:
        bool: True = spreadcycle; False = stealthchop
    """
    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)

    gconf = GConf(gconf)
    return gconf.en_spreadcycle



def set_spreadcycle(self,en_spread):
    """enables spreadcycle (1) or stealthchop (0)

    Args:
      en_spread (bool): true to enable spreadcycle; false to enable stealthchop

    """
    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)

    gconf = GConf(gconf)
    gconf.en_spreadcycle = en_spread
    gconf_int = gconf.serialise()

    self.tmc_uart.write_reg_check(TmcRegAddr.GCONF, gconf_int)



def get_interpolation(self):
    """return whether the tmc inbuilt interpolation is active

    Returns:
        en (bool): true if internal µstep interpolation is enabled
    """
    chopconf = self.tmc_uart.read_int(TmcRegAddr.CHOPCONF)
    chopconf = ChopConf(chopconf)

    return chopconf.intpol



def set_interpolation(self, en):
    """enables the tmc inbuilt interpolation of the steps to 256 µsteps

    Args:
        en (bool): true to enable internal µstep interpolation
    """
    chopconf = self.tmc_uart.read_int(TmcRegAddr.CHOPCONF)
    chopconf = ChopConf(chopconf)

    chopconf.intpol = en
    chopconf_int = chopconf.serialise()
    self.tmc_uart.write_reg_check(TmcRegAddr.CHOPCONF, chopconf_int)



def get_toff(self):
    """returns the TOFF register value

    Returns:
        int: TOFF register value
    """
    chopconf = self.tmc_uart.read_int(TmcRegAddr.CHOPCONF)
    chopconf = ChopConf(chopconf)

    return chopconf.toff



def set_toff(self, toff):
    """Sets TOFF register to value

    Args:
        toff (uint8_t): value of toff (must be a four-bit value)
    """
    chopconf = self.tmc_uart.read_int(TmcRegAddr.CHOPCONF)
    chopconf = ChopConf(chopconf)

    chopconf.toff = toff
    chopconf_int = chopconf.serialise()
    self.tmc_uart.write_reg_check(TmcRegAddr.CHOPCONF, chopconf_int)



def read_microstepping_resolution(self):
    """returns the current native microstep resolution (1-256)
    this reads the value from the driver register

    Returns:
        int: µstep resolution
    """
    chopconf = self.tmc_uart.read_int(TmcRegAddr.CHOPCONF)
    chopconf = ChopConf(chopconf)

    self._msres = chopconf.convert_reg_to_mres()
    self._steps_per_rev = self._fullsteps_per_rev * self._msres

    return self._msres



def get_microstepping_resolution(self):
    """returns the current native microstep resolution (1-256)
    this returns the cached value from this module

    Returns:
        int: µstep resolution
    """
    return self._msres



def set_microstepping_resolution(self, msres):
    """sets the current native microstep resolution (1,2,4,8,16,32,64,128,256)

    Args:
        msres (int): µstep resolution; has to be a power of 2 or 1 for fullstep
    """
    chopconf = self.tmc_uart.read_int(TmcRegAddr.CHOPCONF)
    chopconf = ChopConf(chopconf)

    chopconf.convert_mres_to_reg(msres)
    chopconf_int = chopconf.serialise()
    self.tmc_uart.write_reg_check(TmcRegAddr.CHOPCONF, chopconf_int)

    self._msres = msres
    self._steps_per_rev = self._fullsteps_per_rev * self._msres

    self.set_mstep_resolution_reg_select(True)

    return True



def set_mstep_resolution_reg_select(self, en):
    """sets the register bit "mstep_reg_select" to 1 or 0 depending to the given value.
    this is needed to set the microstep resolution via UART
    this method is called by "set_microstepping_resolution"

    Args:
        en (bool): true to set µstep resolution via UART
    """
    gconf = self.tmc_uart.read_int(TmcRegAddr.GCONF)

    gconf = GConf(gconf)
    gconf.mstep_reg_select = en
    gconf_int = gconf.serialise()

    self.tmc_uart.write_reg_check(TmcRegAddr.GCONF, gconf_int)



def get_interface_transmission_counter(self):
    """reads the interface transmission counter from the tmc register
    this value is increased on every succesfull write access
    can be used to verify a write access

    Returns:
        int: 8bit IFCNT Register
    """
    ifcnt = self.tmc_uart.read_int(TmcRegAddr.IFCNT)
    self.tmc_logger.log(f"Interface Transmission Counter: {ifcnt}", Loglevel.INFO)
    return ifcnt



def get_tstep(self):
    """reads the current tstep from the driver register

    Returns:
        int: TStep time
    """
    tstep = self.tmc_uart.read_int(TmcRegAddr.TSTEP)
    return tstep



def set_vactual(self, vactual):
    """sets the register bit "VACTUAL" to to a given value
    VACTUAL allows moving the motor by UART control.
    It gives the motor velocity in +-(2^23)-1 [μsteps / t]
    0: Normal operation. Driver reacts to STEP input

    Args:
        vactual (int): value for VACTUAL
    """
    self.tmc_uart.write_reg_check(TmcRegAddr.VACTUAL, vactual)



def get_stallguard_result(self):
    """return the current stallguard result
    its will be calculated with every fullstep
    higher values means a lower motor load

    Returns:
        sg_result (int): StallGuard Result
    """
    sg_result = self.tmc_uart.read_int(TmcRegAddr.SG_RESULT)
    return sg_result



def set_stallguard_threshold(self, threshold):
    """sets the register bit "SGTHRS" to to a given value
    this is needed for the stallguard interrupt callback
    SG_RESULT becomes compared to the double of this threshold.
    SG_RESULT ≤ SGTHRS*2

    Args:
        threshold (int): value for SGTHRS
    """
    self.tmc_logger.log(f"sgthrs {bin(threshold)}", Loglevel.INFO)

    self.tmc_logger.log("writing sgthrs", Loglevel.INFO)
    self.tmc_uart.write_reg_check(TmcRegAddr.SGTHRS, threshold)



def set_coolstep_threshold(self, threshold):
    """This  is  the  lower  threshold  velocity  for  switching
    on  smart energy CoolStep and StallGuard to DIAG output. (unsigned)

    Args:
        threshold (int): threshold velocity for coolstep
    """
    self.tmc_logger.log(f"tcoolthrs {bin(threshold)}", Loglevel.INFO)

    self.tmc_logger.log("writing tcoolthrs", Loglevel.INFO)
    self.tmc_uart.write_reg_check(TmcRegAddr.TCOOLTHRS, threshold)



def get_microstep_counter(self):
    """returns the current Microstep counter.
    Indicates actual position in the microstep table for CUR_A

    Returns:
        int: current Microstep counter
    """
    mscnt = self.tmc_uart.read_int(TmcRegAddr.MSCNT)
    return mscnt



def get_microstep_counter_in_steps(self, offset=0):
    """returns the current Microstep counter.
    Indicates actual position in the microstep table for CUR_A

    Args:
        offset (int): offset in steps (Default value = 0)

    Returns:
        step (int): current Microstep counter convertet to steps
    """
    step = (self.get_microstep_counter()-64)*(self._msres*4)/1024
    step = (4*self._msres)-step-1
    step = round(step)
    return step+offset
