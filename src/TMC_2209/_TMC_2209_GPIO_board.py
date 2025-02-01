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
from importlib import import_module
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


class BaseGPIOWrapper:
    """Base class for GPIO implementations"""
    def init(self, gpio_mode=None):
        raise NotImplementedError

    def deinit(self):
        raise NotImplementedError

    def gpio_setup(self, pin, mode, initial=Gpio.LOW, pull_up_down=GpioPUD.PUD_OFF):
        raise NotImplementedError

    def gpio_cleanup(self, pin):
        raise NotImplementedError

    def gpio_input(self, pin):
        raise NotImplementedError

    def gpio_output(self, pin, value):
        raise NotImplementedError

    def gpio_add_event_detect(self, pin, callback):
        raise NotImplementedError

    def gpio_remove_event_detect(self, pin):
        raise NotImplementedError

class BaseRPiGPIOWrapper(BaseGPIOWrapper):
    """RPI.GPIO implementation"""

    def init(self, gpio_mode=None):
        self.GPIO.setwarnings(False)
        if gpio_mode is None:
            gpio_mode = self.GPIO.BCM
        self.GPIO.setmode(gpio_mode)

    def deinit(self):
        self.GPIO.cleanup()

    def gpio_setup(self, pin, mode, initial=Gpio.LOW, pull_up_down=GpioPUD.PUD_OFF):
        initial = int(initial)
        pull_up_down = int(pull_up_down)
        mode = int(mode)
        if mode == GpioMode.OUT:
            self.GPIO.setup(pin, mode, initial=initial)
        else:
            self.GPIO.setup(pin, mode, pull_up_down=pull_up_down)

    def gpio_cleanup(self, pin):
        self.GPIO.cleanup(pin)

    def gpio_input(self, pin):
        return self.GPIO.input(pin)

    def gpio_output(self, pin, value):
        self.GPIO.output(pin, value)

    def gpio_add_event_detect(self, pin, callback):
        self.GPIO.add_event_detect(pin, self.GPIO.RISING, callback=callback, bouncetime=300)

    def gpio_remove_event_detect(self, pin):
       self.GPIO.remove_event_detect(pin)

class MockGPIOWrapper(BaseRPiGPIOWrapper):
    """Mock GPIO implementation"""

    def __init__(self):
        self.GPIO = import_module('Mock.GPIO')
        dependencies_logger.log("using Mock.GPIO for GPIO mocking", Loglevel.INFO)

class RPiGPIOWrapper(BaseRPiGPIOWrapper):
    """Raspberry Pi GPIO implementation"""

    def __init__(self):
        self.GPIO = import_module('RPi.GPIO')
        dependencies_logger.log("using RPi.GPIO for GPIO control", Loglevel.INFO)

class GpiozeroWrapper(BaseRPiGPIOWrapper):
    """Raspberry Pi 5 GPIO implementation"""

    def __init__(self):
        self.gpiozero = import_module('gpiozero')
        dependencies_logger.log("using gpiozero for GPIO control", Loglevel.INFO)
        self._gpios = [None] * 200

    def init(self, gpio_mode=None):
        pass

    def deinit(self):
        pass

    def gpio_setup(self, pin, mode, initial=Gpio.LOW, pull_up_down=GpioPUD.PUD_OFF):
        if mode == GpioMode.OUT:
            self._gpios[pin] = self.gpiozero.DigitalOutputDevice(pin)
        else:
            self._gpios[pin] = self.gpiozero.DigitalInputDevice(pin)

    def gpio_cleanup(self, pin):
        self._gpios[pin].close()

    def gpio_input(self, pin):
        return self._gpios[pin].value

    def gpio_output(self, pin, value):
        self._gpios[pin].value = value

    def gpio_add_event_detect(self, pin, callback):
        self._gpios[pin].when_activated = callback

    def gpio_remove_event_detect(self, pin):
        if self._gpios[pin].when_activated is not None:
            self._gpios[pin].when_activated = None

class JetsonGPIOWrapper(BaseRPiGPIOWrapper):

    def __init__(self):
        self.GPIO = import_module('Jetson.GPIO')
        dependencies_logger.log("using Jetson.GPIO for GPIO control", Loglevel.INFO)

class peripheryWrapper(BaseRPiGPIOWrapper):
    """Luckfox Pico GPIO implementation"""
    def __init__(self):
        self.periphery = import_module('periphery')
        dependencies_logger.log("using periphery for GPIO control", Loglevel.INFO)
        self._gpios = [None] * 200

    def init(self, gpio_mode=None):
        pass

    def deinit(self):
        pass

    def gpio_setup(self, pin, mode, initial=Gpio.LOW, pull_up_down=GpioPUD.PUD_OFF):
        mode = 'out' if (mode == GpioMode.OUT) else 'in'
        self._gpios[pin] = self.periphery.GPIO(pin, mode)

    def gpio_cleanup(self, pin):
        self._gpios[pin].close()

    def gpio_input(self, pin):
        return self._gpios[pin].read()

    def gpio_output(self, pin, value):
        self._gpios[pin].write(bool(value))

class OPiGPIOWrapper(BaseRPiGPIOWrapper):
    """Orange Pi GPIO implementation"""

    def __init__(self):
        self.GPIO = import_module('OPi.GPIO')
        dependencies_logger.log("using OPi.GPIO for GPIO control", Loglevel.INFO)


# Determine the board and instantiate the appropriate GPIO class
def get_board_model_name():
    if not exists('/proc/device-tree/model'):
        return "mock"
    with open('/proc/device-tree/model', encoding="utf-8") as f:
        return f.readline().lower()

def handle_module_not_found_error(err, board_name, module_name, install_link):
    dependencies_logger.log(
        (f"ModuleNotFoundError: {err}\n"
         f"Board is {board_name} but module {module_name} isn't installed.\n"
         f"Follow the installation instructions in the link below to resolve the issue:\n"
         f"{install_link}\n"
         "Exiting..."),
        Loglevel.ERROR)
    raise

def handle_import_error(err, board_name, module_name, install_link):
    dependencies_logger.log(
        (f"ImportError: {err}\n"
         f"Board is {board_name} but module {module_name} isn't installed.\n"
         f"Follow the installation instructions in the link below to resolve the issue:\n"
         f"{install_link}\n"
         "Exiting..."),
        Loglevel.ERROR)
    raise

def initialize_gpio():
    model = get_board_model_name()
    dependencies_logger.log(f"Board model: {model}", Loglevel.INFO)
    if model == "mock":
        return MockGPIOWrapper(), Board.UNKNOWN

    if "raspberry pi 5" in model:
        try:
            return GpiozeroWrapper(), Board.RASPBERRY_PI5
        except ModuleNotFoundError as err:
            handle_module_not_found_error(err, "Raspberry PI 5", "gpiozero", "https://gpiozero.readthedocs.io/en/stable/installing.html")
        except ImportError as err:
            handle_import_error(err, "Raspberry PI 5", "gpiozero", "https://gpiozero.readthedocs.io/en/stable/installing.html")
    elif "raspberry" in model:
        try:
            return RPiGPIOWrapper(), Board.RASPBERRY_PI
        except ModuleNotFoundError as err:
            handle_module_not_found_error(err, "Raspberry PI", "RPi.GPIO", "https://sourceforge.net/p/raspberry-gpio-python/wiki/install")
        except ImportError as err:
            handle_import_error(err, "Raspberry PI", "RPi.GPIO", "https://sourceforge.net/p/raspberry-gpio-python/wiki/install")
    elif "jetson" in model:
        try:
            return JetsonGPIOWrapper(), Board.NVIDIA_JETSON
        except ModuleNotFoundError as err:
            handle_module_not_found_error(err, "Nvidia Jetson", "jetson-gpio", "https://github.com/NVIDIA/jetson-gpio")
        except ImportError as err:
            handle_import_error(err, "Nvidia Jetson", "jetson-gpio", "https://github.com/NVIDIA/jetson-gpio")
    elif "luckfox" in model:
        try:
            return peripheryWrapper(), Board.LUCKFOX_PICO
        except ModuleNotFoundError as err:
            handle_module_not_found_error(err, "Luckfox Pico", "periphery", "https://github.com/vsergeev/python-periphery")
        except ImportError as err:
            handle_import_error(err, "Luckfox Pico", "periphery", "https://github.com/vsergeev/python-periphery")
    elif "orange" in model:
        try:
            return OPiGPIOWrapper(), Board.ORANGE_PI
        except ModuleNotFoundError as err:
            handle_module_not_found_error(err, "Orange Pi", "OPi.GPIO", "https://github.com/rm-hull/OPi.GPIO")
    else:
        dependencies_logger.log(
            "The board is not recognized. Trying import default RPi.GPIO module...",
            Loglevel.INFO)
        try:
            return RPiGPIOWrapper(), Board.UNKNOWN
        except ImportError:
            return MockGPIOWrapper(), Board.UNKNOWN

TMC_gpio, BOARD = initialize_gpio()
