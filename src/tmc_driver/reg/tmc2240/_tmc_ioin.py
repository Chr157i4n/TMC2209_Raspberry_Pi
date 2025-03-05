#pylint: disable=too-many-instance-attributes
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""
INPUT register
"""

from .bitfields import _tmc_224x_ioin as bit
from ._tmc_reg_addr import *
from .._tmc_reg import *


class IOIN(TmcReg):
    """INPUT register"""

    version     : int
    silicon_rv  : int
    adc_err     : bool
    ext_clk     : bool
    ext_res_det : bool
    output      : bool
    comp_b1_b2  : bool
    comp_a1_a2  : bool
    comp_b      : bool
    comp_a      : bool
    uart_en     : bool
    encn        : bool
    drv_enn     : bool
    enca        : bool
    encb        : bool
    dir         : bool
    step        : bool

    def __init__(self, data:int = None):
        """Initialises the object with the given register value

        Args:
            data (int): register value
        """
        self.addr = TmcRegAddr.IOIN
        if data is not None:
            self.deserialise(data)


    def deserialise(self, data: int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.version = data >> bit.version_bp & bit.version_bm
        self.silicon_rv = data >> bit.silicon_rv_bp & bit.silicon_rv_bm
        self.adc_err = bool(data >> bit.adc_err_bp & bit.adc_err_bm)
        self.ext_clk = bool(data >> bit.ext_clk_bp & bit.ext_clk_bm)
        self.ext_res_det = bool(data >> bit.ext_res_det_bp & bit.ext_res_det_bm)
        self.output = bool(data >> bit.output_bp & bit.output_bm)
        self.comp_b1_b2 = bool(data >> bit.comp_b1_b2_bp & bit.comp_b1_b2_bm)
        self.comp_a1_a2 = bool(data >> bit.comp_a1_a2_bp & bit.comp_a1_a2_bm)
        self.comp_b = bool(data >> bit.comp_b_bp & bit.comp_b_bm)
        self.comp_a = bool(data >> bit.comp_a_bp & bit.comp_a_bm)
        self.uart_en = bool(data >> bit.uart_en_bp & bit.uart_en_bm)
        self.encn = bool(data >> bit.encn_bp & bit.encn_bm)
        self.drv_enn = bool(data >> bit.drv_enn_bp & bit.drv_enn_bm)
        self.enca = bool(data >> bit.enca_bp & bit.enca_bm)
        self.encb = bool(data >> bit.encb_bp & bit.encb_bm)
        self.dir = bool(data >> bit.dir_bp & bit.dir_bm)
        self.step = bool(data >> bit.step_bp & bit.step_bm)


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        raise NotImplementedError


    def log(self, logger: TmcLogger):
        """Logs the register values

        Args:
            logger (TmcLogger): Logger
        """
        logger.log(f"STEP: {self.step}")
        logger.log(f"DIR: {self.dir}")
        logger.log(f"Version: {self.version}")
