#pylint: disable=too-many-instance-attributes
#pylint: disable=unused-import
"""
Register module
"""

import typing
from .._tmc_logger import TmcLogger, Loglevel
from ..com._tmc_com import TmcCom


class TmcReg():
    """Register class"""

    _addr: hex
    _name: str
    _tmc_com: TmcCom
    _reg_map: typing.List
    _data_int: int


    @property
    def addr(self) -> hex:
        """addr property"""
        return self._addr

    @property
    def name(self) -> str:
        """name property"""
        return self._name

    @property
    def reg_map(self) -> typing.List:
        """reg_map property"""
        return self._reg_map


    def __init__(self, address:hex, name:str, tmc_com:TmcCom, reg_map:typing.List):
        """Constructor"""
        self._addr = address
        self._name = name
        self._tmc_com = tmc_com
        self._reg_map = reg_map


    def deserialise(self, data:int):
        """Deserialises the register value

        Args:
            data (int): register value
        """
        self._data_int = data

        for reg in self._reg_map:
            name, pos, mask, _, _, _ = reg
            value = data >> pos & mask
            setattr(self, name, reg[3](value))


    def serialise(self) -> int:
        """Serialises the object to a register value

        Returns:
            int: register value
        """
        data = 0

        for reg in self._reg_map:
            name, pos, mask, _, _, _ = reg
            value = getattr(self, name)
            data |= (int(value) & mask) << pos

        return data


    def log(self, logger: TmcLogger):
        """log this register"""
        logger.log(f"{self._name} | {hex(self._addr)} | {bin(self._data_int)}")

        for reg in self._reg_map:
            name, _, _, _, conv_func, unit = reg
            value = getattr(self, name)
            log_string = f"  {name:<20}{value:<10}"
            if conv_func is not None:
                log_string += f" {conv_func()} {unit}"
            logger.log(log_string)


    def read(self):
        """read this register"""
        data = self._tmc_com.read_int(self._addr)
        self.deserialise(data)


    def write(self):
        """write this register"""
        data = self.serialise()
        self._tmc_com.write_reg(self._addr, data)


    def write_check(self):
        """write this register and checks that the write was successful"""
        data = self.serialise()
        self._tmc_com.write_reg_check(self._addr, data)


    def modify(self, name:str, value):
        """modify a register value

        Args:
            name (str): register name
            value: new value
        """
        self.read()
        setattr(self, name, value)
        self.write_check()
