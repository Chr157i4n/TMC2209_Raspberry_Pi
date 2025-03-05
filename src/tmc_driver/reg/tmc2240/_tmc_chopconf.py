#pylint: disable=too-many-instance-attributes
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
"""
Chopper Configuration register
"""

import math
from .bitfields import _tmc_224x_chopconf as bit
from ._tmc_reg_addr import *
from .._tmc_reg import *


class ChopConf(TmcReg):
    """Chopper Configuration register"""

    diss2vs         : bool
    diss2g          : bool
    dedge           : bool
    intpol          : bool
    mres            : int
    tpdf            : int
    vhighchm        : bool
    vhighfs         : bool
    tbl             : int
    chm             : int
    disfdcc         : bool
    fd3             : bool
    hend_offset     : int
    hstrt_tfd210    : int
    toff            : int


    def __init__(self, data:int = None):
        """Initialises the object with the given register value

        Args:
            data (int): register value
        """
        self.addr = TmcRegAddr.CHOPCONF
        if data is not None:
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
        self.tpdf = int(data >> bit.tpdf_bp & bit.tpdf_bm)
        self.vhighchm = bool(data >> bit.vhighchm_bp & bit.vhighchm_bm)
        self.vhighfs = bool(data >> bit.vhighfs_bp & bit.vhighfs_bm)
        self.tbl = int(data >> bit.tbl_bp & bit.tbl_bm)
        self.chm = int(data >> bit.chm_bp & bit.chm_bm)
        self.disfdcc = bool(data >> bit.disfdcc_bp & bit.disfdcc_bm)
        self.fd3 = bool(data >> bit.fd3_bp & bit.fd3_bm)
        self.hend_offset = int(data >> bit.hend_offset_bp & bit.hend_offset_bm)
        self.hstrt_tfd210 = int(data >> bit.hstrt_tfd210_bp & bit.hstrt_tfd210_bm)
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
        data |= self.tpdf << bit.tpdf_bp
        data |= int(self.vhighchm) << bit.vhighchm_bp
        data |= int(self.vhighfs) << bit.vhighfs_bp
        data |= self.tbl << bit.tbl_bp
        data |= self.chm << bit.chm_bp
        data |= int(self.disfdcc) << bit.disfdcc_bp
        data |= int(self.fd3) << bit.fd3_bp
        data |= self.hend_offset << bit.hend_offset_bp
        data |= self.hstrt_tfd210 << bit.hstrt_tfd210_bp
        data |= self.toff << bit.toff_bp

        return data


    def log(self, logger: TmcLogger):
        """Logs the object

        Args:
            logger (TmcLogger): logger
        """
        logger.log("chopconf:")
        logger.log(f"diss2vs: {self.diss2vs}")
        logger.log(f"diss2g: {self.diss2g}")
        logger.log(f"dedge: {self.dedge}")
        logger.log(f"intpol: {self.intpol}")
        logger.log(f"mres: {self.mres} | {self.convert_reg_to_mres()} µsteps")
        logger.log(f"vsense: {self.vsense}")
        logger.log(f"tbl: {self.tbl}")
        logger.log(f"hend: {self.hend}")
        logger.log(f"hstrt: {self.hstrt}")
        logger.log(f"toff: {self.toff}")


    def convert_mres_to_reg(self, mres: int):
        """converts the µstep resolution to the corresponding register value

        Args:
            mres (int): µstep resolution
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
