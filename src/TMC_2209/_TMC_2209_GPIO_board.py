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
from enum import Enum, IntEnum
from ._TMC_2209_logger import TMC_logger, Loglevel

class Board(Enum):
    """board"""
    UNKNOWN = 0
    RASPBERRY_PI = 1 # all except Pi 5
    RASPBERRY_PI5 = 2
    NVIDIA_JETSON = 3

class Gpio(IntEnum):
    """GPIO value"""
    LOW = 0
    HIGH = 1

class GpioMode(IntEnum):
    """GPIO mode"""
    OUT = 0
    IN = 1

class GpioPUD(IntEnum):
    """Pull up Down"""
    PUD_OFF = 20
    PUD_UP = 22
    PUD_DOWN = 21


    


BOARD = Board.UNKNOWN
dependencies_logger = TMC_logger(Loglevel.DEBUG, "DEPENDENCIES")

if not exists('/proc/device-tree/model'):
    from Mock import GPIO
else:
    with open('/proc/device-tree/model', encoding="utf-8") as f:
        model = f.readline().lower()
        if "raspberry pi 5" in model:
            try:
                from RPi import GPIO # TODO use gpiozero or a different lib for Pi5
                BOARD = Board.RASPBERRY_PI5
            except ModuleNotFoundError as err:
                dependencies_logger.log(
                    (f"ModuleNotFoundError: {err}\n" # TODO change text
                    "Board is Raspberry PI 5 but module RPi.GPIO isn`t installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://sourceforge.net/p/raspberry-gpio-python/wiki/install/\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
            except ImportError as err:
                dependencies_logger.log(
                    (f"ImportError: {err}\n"
                    "Board is Raspberry PI 5 but module RPi.GPIO isn`t installed.\n" # TODO change text
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://sourceforge.net/p/raspberry-gpio-python/wiki/install/\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
        elif "raspberry" in model:
            try:
                from RPi import GPIO
                BOARD = Board.RASPBERRY_PI
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
                BOARD = Board.NVIDIA_JETSON
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
            BOARD = Board.UNKNOWN
            try:
                from RPi import GPIO
            except ImportError:
                from Mock import GPIO

class TMC_gpio: 

    def init(gpio_mode=GPIO.BCM):
        GPIO.setwarnings(False)
        if gpio_mode == None:
            gpio_mode = GPIO.BCM
        GPIO.setmode(gpio_mode)

    def deinit():
        pass

    def gpio_setup(pin, mode, initial = Gpio.LOW, pull_up_down = GpioPUD.PUD_OFF):
        GPIO.setup(pin, int(mode), initial=int(initial), pull_up_down=int(pull_up_down))

    def gpio_cleanup(pin):
        GPIO.cleanup(pin)

    def gpio_input(pin):
        return 0

    def gpio_output(pin, value):
        """function to switch a gpio output
        """
        GPIO.output(pin, value)

    def gpio_remove_event_detect(pin):
        GPIO.remove_event_detect(pin)

    def gpio_add_event_detect(pin, callback):
        GPIO.add_event_detect(pin, GPIO.RISING, callback=callback,
                              bouncetime=300)
