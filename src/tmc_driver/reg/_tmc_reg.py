#pylint: disable=too-many-instance-attributes
#pylint: disable=unused-import
"""
Register module
"""

from .._tmc_logger import TmcLogger, Loglevel
from ._tmc_220x_reg_addr import TmcRegAddr


class TmcReg():
    """Register class"""

    addr: TmcRegAddr
    data: int


    def deserialise(self, data:int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        raise NotImplementedError


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        raise NotImplementedError


    def log(self, logger: TmcLogger):
        """log this register"""
        raise NotImplementedError


    def read(self, tmc_com):
        """read this register"""
        data = tmc_com.read_int(self.addr)
        self.deserialise(data)


    def write(self, tmc_com):
        """write this register"""
        data = self.serialise()
        tmc_com.write_reg(self.addr, data)
