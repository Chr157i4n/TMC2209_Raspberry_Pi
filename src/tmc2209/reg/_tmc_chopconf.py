#pylint: disable=too-many-instance-attributes
"""
Chopper Configuration register
"""

import math
from . import _tmc_2209_reg as reg
from .._tmc_logger import TMC_logger



class ChopConf():
    """Chopper Configuration register"""

    data: int

    diss2vs: bool
    diss2g: bool
    dedge: bool
    intpol: bool
    mres: int

    vsense: bool
    tbl: int
    hend: int
    hstrt: int
    toff: int


    def __init__(self, data: int):
        """Initialises the object with the given register value

        Args:
            data (int): register value
        """
        self.deserialise(data)


    def deserialise(self, data: int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        self.data = data

        self.diss2vs = bool(data >> reg.chopconf_diss2vs_bp & reg.chopconf_diss2vs_bm)
        self.diss2g = bool(data >> reg.chopconf_diss2g_bp & reg.chopconf_diss2g_bm)
        self.dedge = bool(data >> reg.chopconf_dedge_bp & reg.chopconf_dedge_bm)
        self.intpol = bool(data >> reg.chopconf_intpol_bp & reg.chopconf_intpol_bm)
        self.mres = int(data >> reg.chopconf_mres_bp & reg.chopconf_mres_bm)
        self.vsense = bool(data >> reg.chopconf_vsense_bp & reg.chopconf_vsense_bm)
        self.tbl = int(data >> reg.chopconf_tbl_bp & reg.chopconf_tbl_bm)
        self.hend = int(data >> reg.chopconf_hend_bp & reg.chopconf_hend_bm)
        self.hstrt = int(data >> reg.chopconf_hstrt_bp & reg.chopconf_hstrt_bm)
        self.toff = int(data >> reg.chopconf_toff_bp & reg.chopconf_toff_bm)


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        data |= int(self.diss2vs) << reg.chopconf_diss2vs_bp
        data |= int(self.diss2g) << reg.chopconf_diss2g_bp
        data |= int(self.dedge) << reg.chopconf_dedge_bp
        data |= int(self.intpol) << reg.chopconf_intpol_bp
        data |= self.mres << reg.chopconf_mres_bp
        data |= int(self.vsense) << reg.chopconf_vsense_bp
        data |= self.tbl << reg.chopconf_tbl_bp
        data |= self.hend << reg.chopconf_hend_bp
        data |= self.hstrt << reg.chopconf_hstrt_bp
        data |= self.toff << reg.chopconf_toff_bp

        return data


    def log(self, logger: TMC_logger):
        """Logs the object

        Args:
            logger (TMC_logger): logger
        """
        logger.log("chopconf:")
        logger.log(f"diss2vs: {self.diss2vs}")
        logger.log(f"diss2g: {self.diss2g}")
        logger.log(f"dedge: {self.dedge}")
        logger.log(f"intpol: {self.intpol}")
        logger.log(f"mres: {self.mres} | {self.convert_reg_to_mres()}")
        logger.log(f"vsense: {self.vsense}")
        logger.log(f"tbl: {self.tbl}")
        logger.log(f"hend: {self.hend}")
        logger.log(f"hstrt: {self.hstrt}")
        logger.log(f"toff: {self.toff}")


    def convert_mres_to_reg(self, mres: int):
        """converts the µstep resolution to the corresponding register value

        Args:
            msres (int): µstep resolution
        """
        mres_reg = int(math.log(mres, 2))
        mres_reg = 8 - mres_reg
        self.mres = mres_reg


    def convert_reg_to_mres(self) -> int:
        """converts the register value to the corresponding µstep resolution

        Returns:
            int: µstep resolution
        """
        return int(math.pow(2, 8 - self.mres))
