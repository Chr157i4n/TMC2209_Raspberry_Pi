"""
Many boards have RaspberryPI-compatible PinOut,
but require to import special GPIO module instead RPI.GPIO

This module determines the type of board
and installs the corresponding GPIO module
"""

from ._TMC_2209_logger import TMC_logger, Loglevel

dependencies_logger = TMC_logger(Loglevel.DEBUG, "DEPENDENCIES")
with open('/proc/device-tree/model', encoding="utf-8") as f:
    model = f.readline().lower()
    if "raspberry" in model:
        try:
            from RPi import GPIO
            BOARD = "RASPBERRY_PI"
        except ModuleNotFoundError as err:
            dependencies_logger.log(
f'''
ModuleNotFoundError: {err}
Board is Raspberry PI but module RPi.GPIO isn`t installed.
Follow the installation instructions in the link below to resolve the issue:
https://sourceforge.net/p/raspberry-gpio-python/wiki/install/

Exiting...''',
            Loglevel.ERROR)
            raise
        except ImportError as err:
            dependencies_logger.log(
f'''
ImportError: {err}
Board is Raspberry PI but module RPi.GPIO isn`t installed.
Follow the installation instructions in the link below to resolve the issue:
https://sourceforge.net/p/raspberry-gpio-python/wiki/install/

Exiting...''',
            Loglevel.ERROR)
            raise
    elif "nvidia jetson" in model:
        try:
            from Jetson import GPIO
            BOARD = "NVIDIA_JETSON"
        except ModuleNotFoundError as err:
            dependencies_logger.log(
f'''
ModuleNotFoundError: {err}
Board is Nvidia Jetson but module jetson-gpio isn`t installed.
Follow the installation instructions in the link below to resolve the issue:
https://github.com/NVIDIA/jetson-gpio

Exiting...''',
            Loglevel.ERROR)
            raise
        except ImportError as err:
            dependencies_logger.log(
f'''
ImportError: {err}
Board is Nvidia Jetson but module jetson-gpio isn`t installed.
Follow the installation instructions in the link below to resolve the issue:
https://github.com/NVIDIA/jetson-gpio

Exiting...''',
            Loglevel.ERROR)
            raise
    else:
        # just in case
        dependencies_logger.log(
            "The board is not recognized. Trying import default RPi.GPIO module...",
             Loglevel.INFO)
        BOARD = "UNKNOWN"
        try:
            from RPi import GPIO
        except ImportError:
            from Mock import GPIO
