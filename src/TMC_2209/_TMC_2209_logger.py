#pylint: disable=invalid-name
"""
TMC_2209 stepper driver logger module
"""

from enum import Enum



class Loglevel(Enum):
    """loglevel"""
    NONE = 0
    ERROR = 10
    INFO = 20
    DEBUG = 30
    MOVEMENT = 40
    ALL = 100



class TMC_logger:
    """
    TMC_2209_comm

    this class has the function:
    move the motor via STEP/DIR pins
    """
    _loglevel = Loglevel.INFO
    _logprefix = "TMC2209"



    def __init__(self, loglevel = Loglevel.INFO, logprefix = "TMC2209"):
        """
        constructor

            Parameters:
                logprefix (string): new logprefix
                loglevel (enum): level for which to log
        """
        if loglevel is not None:
            self._loglevel = loglevel
        if logprefix is not None:
            self._logprefix = logprefix
        else:
            self._logprefix = "TMC2209"



    def set_logprefix(self, logprefix):
        """
        set the logprefix.

            Parameters:
                logprefix (string): new logprefix
        """
        self._logprefix = logprefix



    def set_loglevel(self, loglevel):
        """
        set the loglevel. See the Enum Loglevel

            Parameters:
                loglevel (enum): level for which to log
        """
        self._loglevel = loglevel



    def log(self, message, loglevel=Loglevel.NONE):
        """
        logs a message

            Parameters:
                message (string): message to log
                loglevel (enum): level for which to log
        """
        if self._loglevel.value >= loglevel.value:
            print(self._logprefix+": " +message)
