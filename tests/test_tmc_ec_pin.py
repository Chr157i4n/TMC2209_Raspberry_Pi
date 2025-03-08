"""
test for _tmc_ec_pin.py
"""

import unittest
from unittest import  mock
from src.tmc_driver._tmc_logger import *
from src.tmc_driver.enable_control._tmc_ec_pin import *


class TestTmcEnableControlPin(unittest.TestCase):
    """TestTmcEnableControlPin"""

    def setUp(self):
        """setUp"""
        self.tmc_ec = TmcEnableControlPin(1)
        self.tmc_logger = TmcLogger()
        self.tmc_ec.init(self.tmc_logger)


    def test_enable(self):
        """test_enable"""
        self.tmc_ec.set_motor_enabled(True)
        self.tmc_ec.set_motor_enabled(False)


if __name__ == '__main__':
    unittest.main()
