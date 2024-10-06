#pylint: disable=unused-import
#pylint: disable=ungrouped-imports
#pylint: disable=unknown-option-value
#pylint: disable=possibly-used-before-assignment
#pylint: disable=used-before-assignment
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

# ------------------------------
# LIB           | BOARD
# ------------------------------
# RPi.GPIO      | Pi4, Pi3 etc.
# Jetson.GPIO   | Nvidia Jetson
# gpiozero      | Pi5
# pheriphery    | Luckfox Pico
# OPi.GPIO      | Orange Pi
# ------------------------------

class Board(Enum):
    """board"""
    UNKNOWN = 0
    RASPBERRY_PI = 1 # all except Pi 5
    RASPBERRY_PI5 = 2
    NVIDIA_JETSON = 3
    LUCKFOX_PICO = 4
    ORANGE_PI = 5

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
                from gpiozero import DigitalOutputDevice, DigitalInputDevice
                BOARD = Board.RASPBERRY_PI5
            except ModuleNotFoundError as err:
                dependencies_logger.log(
                    (f"ModuleNotFoundError: {err}\n"
                    "Board is Raspberry PI 5 but module gpiozero isn't installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://gpiozero.readthedocs.io/en/stable/installing.html\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
            except ImportError as err:
                dependencies_logger.log(
                    (f"ImportError: {err}\n"
                    "Board is Raspberry PI 5 but module gpiozero isn't installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://gpiozero.readthedocs.io/en/stable/installing.html\n"
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
                    "Board is Raspberry PI but module RPi.GPIO isn't installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://sourceforge.net/p/raspberry-gpio-python/wiki/install\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
            except ImportError as err:
                dependencies_logger.log(
                    (f"ImportError: {err}\n"
                    "Board is Raspberry PI but module RPi.GPIO isn't installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://sourceforge.net/p/raspberry-gpio-python/wiki/install\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
        elif "jetson" in model:
            try:
                from Jetson import GPIO
                BOARD = Board.NVIDIA_JETSON
            except ModuleNotFoundError as err:
                dependencies_logger.log(
                    (f"ModuleNotFoundError: {err}\n"
                    "Board is Nvidia Jetson but module jetson-gpio isn't installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://github.com/NVIDIA/jetson-gpio\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
            except ImportError as err:
                dependencies_logger.log(
                    (f"ImportError: {err}\n"
                    "Board is Nvidia Jetson but module jetson-gpio isn't installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://github.com/NVIDIA/jetson-gpio\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
        elif "luckfox pico" in model:
            try:
                from periphery import GPIO
                BOARD = Board.LUCKFOX_PICO
            except ModuleNotFoundError as err:
                dependencies_logger.log(
                    (f"ModuleNotFoundError: {err}\n"
                    "Board is Luckfox Pico but module periphery isn't installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://github.com/vsergeev/python-periphery\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
            except ImportError as err:
                dependencies_logger.log(
                    (f"ImportError: {err}\n"
                    "Board is Luckfox Pico but module periphery isn't installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://github.com/vsergeev/python-periphery\n"
                    "Exiting..."),
                Loglevel.ERROR)
                raise
        elif "orange" in model:
            try:
                from OPi import GPIO
                BOARD = Board.ORANGE_PI
            except ModuleNotFoundError as err:
                dependencies_logger.log(
                    (f"ModuleNotFoundError: {err}\n"
                    "Board is Orange Pi but module OPi.GPIO isn't installed.\n"
                    "Follow the installation instructions in the link below to resolve the issue:\n"
                    "https://github.com/rm-hull/OPi.GPIO\n"
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
    """TMC_gpio class"""

    _gpios = [None] * 200

    @staticmethod
    def init(gpio_mode=None):
        """init gpio library"""
        if BOARD == Board.RASPBERRY_PI5:
            pass
        elif BOARD == Board.LUCKFOX_PICO:
            pass
        elif BOARD == Board.ORANGE_PI:
            pass
        else:
            GPIO.setwarnings(False)
            if gpio_mode is None:
                gpio_mode = GPIO.BCM
            GPIO.setmode(gpio_mode)

    @staticmethod
    def deinit():
        """deinit gpio library"""
        if BOARD == Board.RASPBERRY_PI5:
            pass
        else:
            pass

    @staticmethod
    def gpio_setup(pin, mode, initial = Gpio.LOW, pull_up_down = GpioPUD.PUD_OFF):
        """setup gpio pin"""
        #print(f"{BOARD}, {pin}, {mode}")
        if BOARD == Board.RASPBERRY_PI5:
            if mode == GpioMode.OUT:
                TMC_gpio._gpios[pin] = DigitalOutputDevice(pin)
            else:
                TMC_gpio._gpios[pin] = DigitalInputDevice(pin)
        elif BOARD == Board.LUCKFOX_PICO:
            mode = 'out' if (mode == GpioMode.OUT) else 'in'
            TMC_gpio._gpios[pin] = GPIO(pin, mode)
        else:
            initial = int(initial)
            pull_up_down = int(pull_up_down)
            mode = int(mode)
            if mode == GpioMode.OUT: # TODO: better way to pass different params
                GPIO.setup(pin, mode, initial=initial)
            else:
                GPIO.setup(pin, mode, pull_up_down=pull_up_down)

    @staticmethod
    def gpio_cleanup(pin):
        """cleanup gpio pin"""
        if BOARD == Board.RASPBERRY_PI5:
            TMC_gpio._gpios[pin].close()
        elif BOARD == Board.LUCKFOX_PICO:
            TMC_gpio._gpios[pin].close()
        else:
            GPIO.cleanup(pin)

    @staticmethod
    def gpio_input(pin):
        """get input value of gpio pin"""
        del pin
        return 0 # TODO: implement

    @staticmethod
    def gpio_output(pin, value):
        """set output value of gpio pin"""
        if BOARD == Board.RASPBERRY_PI5:
            TMC_gpio._gpios[pin].value = value
        elif BOARD == Board.LUCKFOX_PICO:
            TMC_gpio._gpios[pin].write(bool(value))
        else:
            GPIO.output(pin, value)

    @staticmethod
    def gpio_add_event_detect(pin, callback):
        """add event detect"""
        if BOARD == Board.RASPBERRY_PI5:
            TMC_gpio._gpios[pin].when_activated = callback
        elif BOARD == Board.LUCKFOX_PICO:
            pass # TODO: implement for stallguard
        else:
            GPIO.add_event_detect(pin, GPIO.RISING, callback=callback,
                                bouncetime=300)

    @staticmethod
    def gpio_remove_event_detect(pin):
        """remove event dectect"""
        if BOARD == Board.RASPBERRY_PI5:
            if TMC_gpio._gpios[pin].when_activated is not None:
                TMC_gpio._gpios[pin].when_activated = None
        elif BOARD == Board.LUCKFOX_PICO:
            pass # TODO: implement for stallguard
        else:
            GPIO.remove_event_detect(pin)
