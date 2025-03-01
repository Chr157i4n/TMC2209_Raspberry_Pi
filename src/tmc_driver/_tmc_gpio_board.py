#pylint: disable=unused-import
#pylint: disable=ungrouped-imports
#pylint: disable=unknown-option-value
#pylint: disable=possibly-used-before-assignment
#pylint: disable=used-before-assignment
#pylint: disable=unnecessary-pass
#pylint: disable=abstract-method
#pylint: disable=no-member
"""
Many boards have RaspberryPI-compatible PinOut,
but require to import special GPIO module instead RPI.GPIO

This module determines the type of board
and import the corresponding GPIO module

Can be extended to support BeagleBone or other boards
"""

import types
from os.path import exists
from enum import Enum, IntEnum
from importlib import import_module
from ._tmc_logger import TmcLogger, Loglevel

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
dependencies_logger = TmcLogger(Loglevel.DEBUG, "DEPENDENCIES")


class BaseGPIOWrapper:
    """Base class for GPIO wrappers"""

    def init(self, gpio_mode=None):
        """initialize GPIO library"""
        raise NotImplementedError

    def deinit(self):
        """deinitialize GPIO library"""
        raise NotImplementedError

    def gpio_setup(self, pin:int, mode:GpioMode, initial:Gpio=Gpio.LOW, pull_up_down:GpioPUD=GpioPUD.PUD_OFF):
        """setup GPIO pin"""
        raise NotImplementedError

    def gpio_cleanup(self, pin:int):
        """cleanup GPIO pin"""
        raise NotImplementedError

    def gpio_input(self, pin:int):
        """read GPIO pin"""
        raise NotImplementedError

    def gpio_output(self, pin:int, value):
        """write GPIO pin"""
        raise NotImplementedError

    def gpio_add_event_detect(self, pin:int, callback:types.FunctionType):
        """add event detect"""
        raise NotImplementedError

    def gpio_remove_event_detect(self, pin:int):
        """remove event detect"""
        raise NotImplementedError

class BaseRPiGPIOWrapper(BaseGPIOWrapper):
    """RPI.GPIO base wrapper"""

    def init(self, gpio_mode=None):
        """initialize GPIO library"""
        self.GPIO.setwarnings(False)
        if gpio_mode is None:
            gpio_mode = self.GPIO.BCM
        self.GPIO.setmode(gpio_mode)

    def deinit(self):
        """deinitialize GPIO library"""
        self.GPIO.cleanup()

    def gpio_setup(self, pin:int, mode:GpioMode, initial:Gpio=Gpio.LOW, pull_up_down:GpioPUD=GpioPUD.PUD_OFF):
        """setup GPIO pin"""
        initial = int(initial)
        pull_up_down = int(pull_up_down)
        mode = int(mode)
        if mode == GpioMode.OUT:
            self.GPIO.setup(pin, mode, initial=initial)
        else:
            self.GPIO.setup(pin, mode, pull_up_down=pull_up_down)

    def gpio_cleanup(self, pin:int):
        """cleanup GPIO pin"""
        self.GPIO.cleanup(pin)

    def gpio_input(self, pin:int):
        """read GPIO pin"""
        return self.GPIO.input(pin)

    def gpio_output(self, pin:int, value):
        """write GPIO pin"""
        self.GPIO.output(pin, value)

    def gpio_add_event_detect(self, pin:int, callback:types.FunctionType):
        """add event detect"""
        self.GPIO.add_event_detect(pin, self.GPIO.RISING, callback=callback, bouncetime=300)

    def gpio_remove_event_detect(self, pin:int):
        """remove event detect"""
        self.GPIO.remove_event_detect(pin)

class MockGPIOWrapper(BaseRPiGPIOWrapper):
    """Mock.GPIO wrapper"""

    def __init__(self):
        """constructor, imports Mock.GPIO"""
        self.GPIO = import_module('Mock.GPIO')
        dependencies_logger.log("using Mock.GPIO for GPIO mocking", Loglevel.INFO)

class RPiGPIOWrapper(BaseRPiGPIOWrapper):
    """RPi.GPIO wrapper"""

    def __init__(self):
        """constructor, imports RPi.GPIO"""
        self.GPIO = import_module('RPi.GPIO')
        dependencies_logger.log("using RPi.GPIO for GPIO control", Loglevel.INFO)

class JetsonGPIOWrapper(BaseRPiGPIOWrapper):
    """Jetson.GPIO wrapper"""

    def __init__(self):
        """constructor, imports Jetson.GPIO"""
        self.GPIO = import_module('Jetson.GPIO')
        dependencies_logger.log("using Jetson.GPIO for GPIO control", Loglevel.INFO)

class OPiGPIOWrapper(BaseRPiGPIOWrapper):
    """OPi.GPIO wrapper"""

    def __init__(self):
        """constructor, imports OPi.GPIO"""
        self.GPIO = import_module('OPi.GPIO')
        dependencies_logger.log("using OPi.GPIO for GPIO control", Loglevel.INFO)

class GpiozeroWrapper(BaseGPIOWrapper):
    """gpiozero GPIO wrapper"""

    def __init__(self):
        """constructor, imports gpiozero"""
        self.gpiozero = import_module('gpiozero')
        dependencies_logger.log("using gpiozero for GPIO control", Loglevel.INFO)
        self._gpios = [None] * 200

    def init(self, gpio_mode=None):
        """initialize GPIO library. pass on gpiozero"""
        pass

    def deinit(self):
        """deinitialize GPIO library. pass on gpiozero"""
        pass

    def gpio_setup(self, pin:int, mode:GpioMode, initial:Gpio=Gpio.LOW, pull_up_down:GpioPUD=GpioPUD.PUD_OFF):
        """setup GPIO pin"""
        if mode == GpioMode.OUT:
            self._gpios[pin] = self.gpiozero.DigitalOutputDevice(pin)
        else:
            self._gpios[pin] = self.gpiozero.DigitalInputDevice(pin)

    def gpio_cleanup(self, pin:int):
        """cleanup GPIO pin"""
        self._gpios[pin].close()

    def gpio_input(self, pin:int):
        """read GPIO pin"""
        return self._gpios[pin].value

    def gpio_output(self, pin:int, value):
        """write GPIO pin"""
        self._gpios[pin].value = value

    def gpio_add_event_detect(self, pin:int, callback:types.FunctionType):
        """add event detect"""
        self._gpios[pin].when_activated = callback

    def gpio_remove_event_detect(self, pin:int):
        """remove event detect"""
        if self._gpios[pin].when_activated is not None:
            self._gpios[pin].when_activated = None

class peripheryWrapper(BaseGPIOWrapper):
    """periphery GPIO wrapper"""

    def __init__(self):
        """constructor, imports periphery"""
        self.periphery = import_module('periphery')
        dependencies_logger.log("using periphery for GPIO control", Loglevel.INFO)
        self._gpios = [None] * 200

    def init(self, gpio_mode=None):
        """initialize GPIO library. pass on periphery"""
        pass

    def deinit(self):
        """deinitialize GPIO library. pass on periphery"""
        pass

    def gpio_setup(self, pin:int, mode:GpioMode, initial:Gpio=Gpio.LOW, pull_up_down:GpioPUD=GpioPUD.PUD_OFF):
        """setup GPIO pin"""
        mode = 'out' if (mode == GpioMode.OUT) else 'in'
        self._gpios[pin] = self.periphery.GPIO(pin, mode)

    def gpio_cleanup(self, pin:int):
        """cleanup GPIO pin"""
        self._gpios[pin].close()

    def gpio_input(self, pin:int):
        """read GPIO pin"""
        return self._gpios[pin].read()

    def gpio_output(self, pin:int, value):
        """write GPIO pin"""
        self._gpios[pin].write(bool(value))


board_mapping = {
    "raspberry pi 5": (GpiozeroWrapper, Board.RASPBERRY_PI5, "gpiozero", "https://gpiozero.readthedocs.io/en/stable/installing.html"),
    "raspberry": (RPiGPIOWrapper, Board.RASPBERRY_PI, "RPi.GPIO", "https://sourceforge.net/p/raspberry-gpio-python/wiki/install"),
    "jetson": (JetsonGPIOWrapper, Board.NVIDIA_JETSON, "jetson-gpio", "https://github.com/NVIDIA/jetson-gpio"),
    "luckfox": (peripheryWrapper, Board.LUCKFOX_PICO, "periphery", "https://github.com/vsergeev/python-periphery"),
    "orange": (OPiGPIOWrapper, Board.ORANGE_PI, "OPi.GPIO", "https://github.com/rm-hull/OPi.GPIO")
}



# Determine the board and instantiate the appropriate GPIO class
def get_board_model_name():
    """get board model name from /proc/device-tree/model file"""
    if not exists('/proc/device-tree/model'):
        return "mock"
    with open('/proc/device-tree/model', encoding="utf-8") as f:
        return f.readline().lower()

def handle_module_not_found_error(err, board_name, module_name, install_link):
    """handle module not found error"""
    dependencies_logger.log(
        (f"ModuleNotFoundError: {err}\n"
         f"Board is {board_name} but module {module_name} isn't installed.\n"
         f"Follow the installation instructions in the link below to resolve the issue:\n"
         f"{install_link}\n"
         "Exiting..."),
        Loglevel.ERROR)
    raise err

def handle_import_error(err, board_name, module_name, install_link):
    """handle import error"""
    dependencies_logger.log(
        (f"ImportError: {err}\n"
         f"Board is {board_name} but module {module_name} isn't installed.\n"
         f"Follow the installation instructions in the link below to resolve the issue:\n"
         f"{install_link}\n"
         "Exiting..."),
        Loglevel.ERROR)
    raise err

def initialize_gpio():
    """initialize GPIO"""
    model = get_board_model_name()
    dependencies_logger.log(f"Board model: {model}", Loglevel.INFO)
    if model == "mock":
        return MockGPIOWrapper(), Board.UNKNOWN


    for key, (wrapper_class, board_enum, module_name, install_link) in board_mapping.items():
        if key in model:
            try:
                return wrapper_class(), board_enum
            except ModuleNotFoundError as err:
                handle_module_not_found_error(err, key.capitalize(), module_name, install_link)
            except ImportError as err:
                handle_import_error(err, key.capitalize(), module_name, install_link)

    dependencies_logger.log(
        "The board is not recognized. Trying import default RPi.GPIO module...",
        Loglevel.INFO)
    try:
        return RPiGPIOWrapper(), Board.UNKNOWN
    except ImportError:
        return MockGPIOWrapper(), Board.UNKNOWN

tmc_gpio, BOARD = initialize_gpio()
