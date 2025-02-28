#pylint: disable=too-many-instance-attributes
"""
Chopper Configuration register
"""

import math
from .bitfields import _tmc_220x_chopconf as bit
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

        self.diss2vs = bool(data >> bit.diss2vs_bp & bit.diss2vs_bm)
        self.diss2g = bool(data >> bit.diss2g_bp & bit.diss2g_bm)
        self.dedge = bool(data >> bit.dedge_bp & bit.dedge_bm)
        self.intpol = bool(data >> bit.intpol_bp & bit.intpol_bm)
        self.mres = int(data >> bit.mres_bp & bit.mres_bm)
        self.vsense = bool(data >> bit.vsense_bp & bit.vsense_bm)
        self.tbl = int(data >> bit.tbl_bp & bit.tbl_bm)
        self.hend = int(data >> bit.hend_bp & bit.hend_bm)
        self.hstrt = int(data >> bit.hstrt_bp & bit.hstrt_bm)
        self.toff = int(data >> bit.toff_bp & bit.toff_bm)


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        data |= int(self.diss2vs) << bit.diss2vs_bp
        data |= int(self.diss2g) << bit.diss2g_bp
        data |= int(self.dedge) << bit.dedge_bp
        data |= int(self.intpol) << bit.intpol_bp
        data |= self.mres << bit.mres_bp
        data |= int(self.vsense) << bit.vsense_bp
        data |= self.tbl << bit.tbl_bp
        data |= self.hend << bit.hend_bp
        data |= self.hstrt << bit.hstrt_bp
        data |= self.toff << bit.toff_bp

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
        mres_bit = int(math.log(mres, 2))
        mres_bit = 8 - mres_bit
        self.mres = mres_bit


    def convert_reg_to_mres(self) -> int:
        """converts the register value to the corresponding µstep resolution

        Returns:
            int: µstep resolution
        """
        return int(math.pow(2, 8 - self.mres))
