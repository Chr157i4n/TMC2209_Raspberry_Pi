#pylint: disable=unused-import
#pylint: disable=ungrouped-imports
"""
Many boards have RaspberryPI-compatible PinOut,
but require to import special GPIO module instead RPI.GPIO

This module determines the type of board
and import the corresponding GPIO module

Can be extended to support BeagleBone or other boards
"""

from os.path import exists
from ._TMC_2209_logger import TMC_logger, Loglevel

BOARD = "UNKNOWN"
dependencies_logger = TMC_logger(Loglevel.DEBUG, "DEPENDENCIES")

if not exists('/proc/device-tree/model'):
    from Mock import GPIO
else:
    with open('/proc/device-tree/model', encoding="utf-8") as f:
        model = f.readline().lower()
        if "raspberry" in model:
            try:
                from RPi import GPIO
                BOARD = "RASPBERRY_PI"
            except ModuleNotFoundError as err:
                dependencies_logger.log(
                    (f"ModuleNotFoundError: {err}\n"
                    "Board is Raspberry PI but module RPi.GPIO isn`t installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://sourceforge.net/p/raspberry-gpio-python/wiki/install/\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
            except ImportError as err:
                dependencies_logger.log(
                    (f"ImportError: {err}\n"
                    "Board is Raspberry PI but module RPi.GPIO isn`t installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://sourceforge.net/p/raspberry-gpio-python/wiki/install/\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
        elif "nvidia jetson" in model:
            try:
                from Jetson import GPIO
                BOARD = "NVIDIA_JETSON"
            except ModuleNotFoundError as err:
                dependencies_logger.log(
                    (f"ModuleNotFoundError: {err}\n"
                    "Board is Nvidia Jetson but module jetson-gpio isn`t installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://github.com/NVIDIA/jetson-gpio\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
            except ImportError as err:
                dependencies_logger.log(
                    (f"ImportError: {err}\n"
                    "Board is Nvidia Jetson but module jetson-gpio isn`t installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://github.com/NVIDIA/jetson-gpio\n"
                    "Exiting..."),
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
