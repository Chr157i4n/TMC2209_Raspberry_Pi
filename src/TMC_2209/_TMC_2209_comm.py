#pylint: disable=invalid-name
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=protected-access
"""
TMC_2209 stepper driver communication module
"""

import math
from ._TMC_2209_logger import Loglevel
from . import _TMC_2209_reg as tmc_reg



def read_drv_status(self):
    """
    read the register Adress "DRV_STATUS" and prints all current setting

        Returns:
            drvstatus (int): 32bit DRV_STATUS Register

    """
    self.tmc_logger.log("---")
    self.tmc_logger.log("DRIVER STATUS:")
    drvstatus =self.tmc_uart.read_int(tmc_reg.DRVSTATUS)
    self.tmc_logger.log(bin(drvstatus), Loglevel.INFO)
    if drvstatus & tmc_reg.stst:
        self.tmc_logger.log("Info: motor is standing still")
    else:
        self.tmc_logger.log("Info: motor is running")

    if drvstatus & tmc_reg.stealth:
        self.tmc_logger.log("Info: motor is running on StealthChop")
    else:
        self.tmc_logger.log("Info: motor is running on SpreadCycle")

    cs_actual = drvstatus & tmc_reg.cs_actual
    cs_actual = cs_actual >> 16
    self.tmc_logger.log("CS actual: "+str(cs_actual))

    if drvstatus & tmc_reg.olb:
        self.tmc_logger.log("Warning: Open load detected on phase B")

    if drvstatus & tmc_reg.ola:
        self.tmc_logger.log("Warning: Open load detected on phase A")

    if drvstatus & tmc_reg.s2vsb:
        self.tmc_logger.log("""Error: Short on low-side MOSFET detected on phase B.
                    The driver becomes disabled""")

    if drvstatus & tmc_reg.s2vsa:
        self.tmc_logger.log("""Error: Short on low-side MOSFET detected on phase A.
                    The driver becomes disabled""")

    if drvstatus & tmc_reg.s2gb:
        self.tmc_logger.log("""Error: Short to GND detected on phase B.
                            The driver becomes disabled.""")

    if drvstatus & tmc_reg.s2ga:
        self.tmc_logger.log("""Error: Short to GND detected on phase A.
                            The driver becomes disabled.""")

    if drvstatus & tmc_reg.ot:
        self.tmc_logger.log("Error: Driver Overheating!")

    if drvstatus & tmc_reg.otpw:
        self.tmc_logger.log("Warning: Driver Overheating Prewarning!")

    self.tmc_logger.log("---")
    return drvstatus



def read_gconf(self):
    """
    read the register Adress "GCONF" and prints all current setting

        Returns:
            gconf (int): 10bit GCONF Register
    """
    self.tmc_logger.log("---")
    self.tmc_logger.log("GENERAL CONFIG")
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    self.tmc_logger.log(bin(gconf), Loglevel.INFO)

    if gconf & tmc_reg.i_scale_analog:
        self.tmc_logger.log("Driver is using voltage supplied to VREF as current reference")
    else:
        self.tmc_logger.log("Driver is using internal reference derived from 5VOUT")
    if gconf & tmc_reg.internal_rsense:
        self.tmc_logger.log("""Internal sense resistors.
                            Use current supplied into VREF as reference.""")
        self.tmc_logger.log("VREF pin internally is driven to GND in this mode.")
        self.tmc_logger.log("This will most likely destroy your driver!!!")
        raise SystemExit
    self.tmc_logger.log("Operation with external sense resistors")
    if gconf & tmc_reg.en_spreadcycle:
        self.tmc_logger.log("SpreadCycle mode enabled")
    else:
        self.tmc_logger.log("StealthChop PWM mode enabled")
    if gconf & tmc_reg.shaft:
        self.tmc_logger.log("Inverse motor direction")
    else:
        self.tmc_logger.log("normal motor direction")
    if gconf & tmc_reg.index_otpw:
        self.tmc_logger.log("INDEX pin outputs overtemperature prewarning flag")
    else:
        self.tmc_logger.log("INDEX shows the first microstep position of sequencer")
    if gconf & tmc_reg.index_step:
        self.tmc_logger.log("INDEX output shows step pulses from internal pulse generator")
    else:
        self.tmc_logger.log("INDEX output as selected by index_otpw")
    if gconf & tmc_reg.mstep_reg_select:
        self.tmc_logger.log("Microstep resolution selected by MSTEP register")
    else:
        self.tmc_logger.log("Microstep resolution selected by pins MS1, MS2")

    self.tmc_logger.log("---")
    return gconf



def read_gstat(self):
    """
    read the register Adress "GSTAT" and prints all current setting

        Returns:
            gstat (int): 3bit GSTAT Register
    """
    self.tmc_logger.log("---")
    self.tmc_logger.log("GSTAT")
    gstat = self.tmc_uart.read_int(tmc_reg.GSTAT)
    self.tmc_logger.log(bin(gstat), Loglevel.INFO)
    if gstat & tmc_reg.reset:
        self.tmc_logger.log("The Driver has been reset since the last read access to GSTAT")
    if gstat & tmc_reg.drv_err:
        self.tmc_logger.log("""The driver has been shut down due to overtemperature or
                    short circuit detection since the last read access""")
    if gstat & tmc_reg.uv_cp:
        self.tmc_logger.log("""Undervoltage on the charge pump.
                            The driver is disabled in this case""")
    self.tmc_logger.log("---")
    return gstat



def clear_gstat(self):
    """
    read the register Adress "GSTAT" and prints all current setting
    """
    self.tmc_logger.log("clearing GSTAT", Loglevel.INFO)
    gstat = self.tmc_uart.read_int(tmc_reg.GSTAT)

    gstat = self.tmc_uart.set_bit(gstat, tmc_reg.reset)
    gstat = self.tmc_uart.set_bit(gstat, tmc_reg.drv_err)

    self.tmc_uart.write_reg_check(tmc_reg.GSTAT, gstat)



def read_ioin(self):
    """
    read the register Adress "IOIN" and prints all current setting

        Returns:
            ioin (int): 10+8bit IOIN Register
    """
    self.tmc_logger.log("---")
    self.tmc_logger.log("INPUTS")
    ioin = self.tmc_uart.read_int(tmc_reg.IOIN)
    self.tmc_logger.log(bin(ioin), Loglevel.INFO)
    if ioin & tmc_reg.io_spread:
        self.tmc_logger.log("spread is high")
    else:
        self.tmc_logger.log("spread is low")

    if ioin & tmc_reg.io_dir:
        self.tmc_logger.log("dir is high")
    else:
        self.tmc_logger.log("dir is low")

    if ioin & tmc_reg.io_step:
        self.tmc_logger.log("step is high")
    else:
        self.tmc_logger.log("step is low")

    if ioin & tmc_reg.io_enn:
        self.tmc_logger.log("en is high")
    else:
        self.tmc_logger.log("en is low")

    self.tmc_logger.log("---")
    return ioin



def read_chopconf(self):
    """
    read the register Adress "CHOPCONF" and prints all current setting

        Returns:
            chopconf (int): 3bit CHOPCONF Register
    """
    self.tmc_logger.log("---")
    self.tmc_logger.log("CHOPPER CONTROL")
    chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
    self.tmc_logger.log(bin(chopconf), Loglevel.INFO)

    self.tmc_logger.log("native "+str(self.get_microstepping_resolution())+" microstep setting")

    if chopconf & tmc_reg.intpol:
        self.tmc_logger.log("interpolation to 256 microsteps")

    if chopconf & tmc_reg.vsense:
        self.tmc_logger.log("1: High sensitivity, low sense resistor voltage")
    else:
        self.tmc_logger.log("0: Low sensitivity, high sense resistor voltage")

    self.tmc_logger.log("---")
    return chopconf



def get_direction_reg(self):
    """
    returns the motor shaft direction: 0 = CCW; 1 = CW

        Returns:
            direction (bool): motor shaft direction: 0 = CCW; 1 = CW
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    return gconf & tmc_reg.shaft



def set_direction_reg(self, direction):
    """
    sets the motor shaft direction to the given value: 0 = CCW; 1 = CW

        Parameters:
            direction (bool): motor shaft direction: 0 = CCW; 1 = CW
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    if direction:
        self.tmc_logger.log("write inverse motor direction", Loglevel.INFO)
        gconf = self.tmc_uart.set_bit(gconf, tmc_reg.shaft)
    else:
        self.tmc_logger.log("write normal motor direction", Loglevel.INFO)
        gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.shaft)
    self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



def get_iscale_analog(self):
    """
    return whether Vref (1) or 5V (0) is used for current scale

        Returns:
            en (bool): whether Vref (1) or 5V (0) is used for current scale
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    return gconf & tmc_reg.i_scale_analog



def set_iscale_analog(self,en):
    """
    sets Vref (1) or 5V (0) for current scale

        Parameters:
            en (bool): whether Vref (1) or 5V (0) is used for current scale
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    if en:
        self.tmc_logger.log("activated Vref for current scale", Loglevel.INFO)
        gconf = self.tmc_uart.set_bit(gconf, tmc_reg.i_scale_analog)
    else:
        self.tmc_logger.log("activated 5V-out for current scale", Loglevel.INFO)
        gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.i_scale_analog)
    self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



def get_vsense(self):
    """
    returns which sense resistor voltage is used for current scaling
    0: Low sensitivity, high sense resistor voltage
    1: High sensitivity, low sense resistor voltage

        Returns:
            en (bool): whether high sensitivity should is used
    """
    chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
    return chopconf & tmc_reg.vsense



def set_vsense(self,en):
    """
    sets which sense resistor voltage is used for current scaling
    0: Low sensitivity, high sense resistor voltage
    1: High sensitivity, low sense resistor voltage

        Parameters:
            en (bool): whether high sensitivity should be used
    """
    chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
    if en:
        self.tmc_logger.log("activated High sensitivity, low sense resistor voltage",
                            Loglevel.INFO)
        chopconf = self.tmc_uart.set_bit(chopconf, tmc_reg.vsense)
    else:
        self.tmc_logger.log("activated Low sensitivity, high sense resistor voltage",
                            Loglevel.INFO)
        chopconf = self.tmc_uart.clear_bit(chopconf, tmc_reg.vsense)
    self.tmc_uart.write_reg_check(tmc_reg.CHOPCONF, chopconf)



def get_internal_rsense(self):
    """
    returns which sense resistor voltage is used for current scaling
    0: Operation with external sense resistors
    1: Internal sense resistors. Use current supplied into
    VREF as reference for internal sense resistor. VREF
    pin internally is driven to GND in this mode.

        Returns:
            en (bool): which sense resistor voltage is used
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    return gconf & tmc_reg.internal_rsense



def set_internal_rsense(self,en):
    """
    sets which sense resistor voltage is used for current scaling
    0: Operation with external sense resistors
    1: Internal sense resistors. Use current supplied into
    VREF as reference for internal sense resistor. VREF
    pin internally is driven to GND in this mode.

        Parameters:
            en (bool):which sense resistor voltage is used; true will propably destroy your tmc
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    if en:
        self.tmc_logger.log("activated internal sense resistors.",
                            Loglevel.INFO)
        self.tmc_logger.log("VREF pin internally is driven to GND in this mode.",
                            Loglevel.INFO)
        self.tmc_logger.log("This will most likely destroy your driver!!!",
                            Loglevel.INFO)
        raise SystemExit
        # gconf = self.tmc_uart.set_bit(gconf, tmc_reg.internal_rsense)
    self.tmc_logger.log("activated operation with external sense resistors",
                        Loglevel.INFO)
    gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.internal_rsense)
    self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



def set_irun_ihold(self, ihold, irun, ihold_delay):
    """
    sets the current scale (CS) for Running and Holding
    and the delay, when to be switched to Holding current

        Parameters:
            ihold (int): 0-31
            irun (int): 0-31
            ihold_delay (int): 0-15
    """
    ihold_irun = 0

    ihold_irun = ihold_irun | ihold << 0
    ihold_irun = ihold_irun | irun << 8
    ihold_irun = ihold_irun | ihold_delay << 16
    self.tmc_logger.log("ihold_irun", Loglevel.INFO)
    self.tmc_logger.log(str(bin(ihold_irun)), Loglevel.INFO)

    self.tmc_logger.log("writing ihold_irun", Loglevel.INFO)
    self.tmc_uart.write_reg_check(tmc_reg.IHOLD_IRUN, ihold_irun)



def set_pdn_disable(self,pdn_disable):
    """
    disables PDN on the UART pin
    0: PDN_UART controls standstill current reduction
    1: PDN_UART input function disabled. Set this bit,
    when using the UART interface!

        Parameters:
            pdn_disable (bool): whether pdn should be disabled
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    if pdn_disable:
        self.tmc_logger.log("enabled PDN_UART", Loglevel.INFO)
        gconf = self.tmc_uart.set_bit(gconf, tmc_reg.pdn_disable)
    else:
        self.tmc_logger.log("disabled PDN_UART", Loglevel.INFO)
        gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.pdn_disable)
    self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



def set_current(self, run_current, hold_current_multiplier = 0.5,
                hold_current_delay = 10, pdn_disable = True):
    """
    sets the current flow for the motor. 

        Parameters:
            run_current (int): current during movement in mA
            hold_current_multiplier (int): current multiplier during standstill
            hold_current_delay (int): delay after standstill after which the current drops
            use_vref (int): most driver have a trimpotentiometer to set the vref. optional
            Vref (int): if you want to use Vref, you need to set the Vref Voltage
            pdn_disable (bool): should be disabled if UART is used
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

    self.tmc_logger.log("cs_irun: " + str(cs_irun), Loglevel.INFO)
    self.tmc_logger.log("CS_IHold: " + str(CS_IHold), Loglevel.INFO)
    self.tmc_logger.log("Delay: " + str(hold_current_delay), Loglevel.INFO)

    # return (float)(CS+1)/32.0 * (vsense() ? 0.180 : 0.325)/(rsense+0.02) / 1.41421 * 1000;
    run_current_actual = (cs_irun+1)/32.0 * (vfs)/(rsense+0.02) / 1.41421 * 1000
    self.tmc_logger.log("actual current: "+str(round(run_current_actual))+" mA",
                        Loglevel.INFO)

    self.set_irun_ihold(CS_IHold, cs_irun, hold_current_delay)

    self.set_pdn_disable(pdn_disable)



def get_spreadcycle(self):
    """
    return whether spreadcycle (1) is active or stealthchop (0)

        Parameters:
            en_spread (bool): true = spreadcycle; false = stealthchop
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    return gconf & tmc_reg.en_spreadcycle



def set_spreadcycle(self,en_spread):
    """
    enables spreadcycle (1) or stealthchop (0)

        Parameters:
            en_spread (bool): true to enable spreadcycle; false to enable stealthchop
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)
    if en_spread:
        self.tmc_logger.log("activated Spreadcycle", Loglevel.INFO)
        gconf = self.tmc_uart.set_bit(gconf, tmc_reg.en_spreadcycle)
    else:
        self.tmc_logger.log("activated Stealthchop", Loglevel.INFO)
        gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.en_spreadcycle)
    self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



def get_interpolation(self):
    """
    return whether the tmc inbuilt interpolation is active

        Returns:
            en (bool): true if internal µstep interpolation is enabled
    """
    chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
    return bool(chopconf & tmc_reg.intpol)



def set_interpolation(self, en):
    """
    enables the tmc inbuilt interpolation of the steps to 256 microsteps

        Parameters:
            en (bool): true to enable internal µstep interpolation
    """
    chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)

    if en:
        chopconf = self.tmc_uart.set_bit(chopconf, tmc_reg.intpol)
    else:
        chopconf = self.tmc_uart.clear_bit(chopconf, tmc_reg.intpol)

    self.tmc_logger.log("writing microstep interpolation setting: "+str(en),
                        Loglevel.INFO)
    self.tmc_uart.write_reg_check(tmc_reg.CHOPCONF, chopconf)



def read_microstepping_resolution(self):
    """
    returns the current native microstep resolution (1-256)
    this reads the value from the driver register

        Returns:
            msres (int): µstep resolution
    """
    chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)

    msresdezimal = chopconf & (tmc_reg.msres0 | tmc_reg.msres1 |
                                tmc_reg.msres2 | tmc_reg.msres3)

    msresdezimal = msresdezimal >> 24
    msresdezimal = 8 - msresdezimal

    self._msres = int(math.pow(2, msresdezimal))

    return self._msres



def get_microstepping_resolution(self):
    """
    returns the current native microstep resolution (1-256)
    this returns the cached value from this module

        Returns:
            msres (int): µstep resolution
    """
    return self._msres



def set_microstepping_resolution(self, msres):
    """
    sets the current native microstep resolution (1,2,4,8,16,32,64,128,256)

        Parameters:
            msres (int): µstep resolution; has to be a power of 2 or 1 for fullstep
    """
    chopconf = self.tmc_uart.read_int(tmc_reg.CHOPCONF)
    #setting all bits to zero
    chopconf = chopconf & (~tmc_reg.msres0 | ~tmc_reg.msres1 |
                            ~tmc_reg.msres2 | ~tmc_reg.msres3)
    msresdezimal = int(math.log(msres, 2))
    msresdezimal = 8 - msresdezimal
    chopconf = int(chopconf) & int(4043309055)
    chopconf = chopconf | msresdezimal <<24

    self.tmc_logger.log("writing "+str(msres)+" microstep setting", Loglevel.INFO)
    self.tmc_uart.write_reg_check(tmc_reg.CHOPCONF, chopconf)

    self._msres = msres

    self.set_mstep_resolution_reg_select(True)

    return True



def set_mstep_resolution_reg_select(self, en):
    """
    sets the register bit "mstep_reg_select" to 1 or 0 depending to the given value.
    this is needed to set the microstep resolution via UART
    this method is called by "set_microstepping_resolution"

        Parameters:
            en (bool): true to set µstep resolution via UART
    """
    gconf = self.tmc_uart.read_int(tmc_reg.GCONF)

    if en is True:
        gconf = self.tmc_uart.set_bit(gconf, tmc_reg.mstep_reg_select)
    else:
        gconf = self.tmc_uart.clear_bit(gconf, tmc_reg.mstep_reg_select)

    self.tmc_logger.log("writing MStep Reg Select: "+str(en), Loglevel.INFO)
    self.tmc_uart.write_reg_check(tmc_reg.GCONF, gconf)



def get_interface_transmission_counter(self):
    """
    reads the interface transmission counter from the tmc register
    this value is increased on every succesfull write access
    can be used to verify a write access

        Returns:
            ifcnt (int): 8bit IFCNT Register
    """
    ifcnt = self.tmc_uart.read_int(tmc_reg.IFCNT)
    self.tmc_logger.log("Interface Transmission Counter: "+str(ifcnt), Loglevel.INFO)
    return ifcnt



def get_tstep(self):
    """
    reads the current tstep from the driver register

        Returns:
            tstep (int): TStep time
    """
    tstep = self.tmc_uart.read_int(tmc_reg.TSTEP)
    return tstep



def set_vactual(self, vactual):
    """
    sets the register bit "VACTUAL" to to a given value
    VACTUAL allows moving the motor by UART control.
    It gives the motor velocity in +-(2^23)-1 [μsteps / t]
    0: Normal operation. Driver reacts to STEP input

        Parameters:
            vactual (int): value for vactual
    """
    self.tmc_uart.write_reg_check(tmc_reg.VACTUAL, vactual)



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

    self.tmc_logger.log("sgthrs", Loglevel.INFO)
    self.tmc_logger.log(str(bin(threshold)), Loglevel.INFO)

    self.tmc_logger.log("writing sgthrs", Loglevel.INFO)
    self.tmc_uart.write_reg_check(tmc_reg.SGTHRS, threshold)



def set_coolstep_threshold(self, threshold):
    """
    This  is  the  lower  threshold  velocity  for  switching  
    on  smart energy CoolStep and StallGuard to DIAG output. (unsigned)

        Parameters:
            threshold (int): threshold  velocity
    """

    self.tmc_logger.log("tcoolthrs", Loglevel.INFO)
    self.tmc_logger.log(str(bin(threshold)), Loglevel.INFO)

    self.tmc_logger.log("writing tcoolthrs", Loglevel.INFO)
    self.tmc_uart.write_reg_check(tmc_reg.TCOOLTHRS, threshold)



def get_microstep_counter(self):
    """
    returns the current Microstep counter.
    Indicates actual position in the microstep table for CUR_A

        Returns:
            mscnt (int): current Microstep counter
    """
    mscnt = self.tmc_uart.read_int(tmc_reg.MSCNT)
    return mscnt



def get_microstep_counter_in_steps(self, offset=0):
    """
    returns the current Microstep counter.
    Indicates actual position in the microstep table for CUR_A

        Parameters:
            offset (int): offset added to the current step counter

        Returns:
            step (int): current Microstep counter convertet to steps
    """
    step = (self.get_microstep_counter()-64)*(self._msres*4)/1024
    step = (4*self._msres)-step-1
    step = round(step)
    return step+offset
