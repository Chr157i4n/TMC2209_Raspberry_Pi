#pylint: disable=missing-function-docstring
#pylint: disable=missing-class-docstring
#pylint: disable=wildcard-import
#pylint: disable=pointless-statement
#pylint: disable=unused-wildcard-import
"""
test for _tmc_uart.py
"""

import unittest
from unittest import  mock
from src.tmc_driver._tmc_logger import *
from src.tmc_driver._tmc_uart import *


class TestTMCUart(unittest.TestCase):
    """TestTMCUart"""

    def setUp(self):
        """setUp"""
        self.tmc_logger = TmcLogger()
        self.tmc_uart = TmcUart(None, 115200, 0, self.tmc_logger)

    def test_read_int(self):
        """test_read_int"""
        self.tmc_uart.ser = 1 # to avoid early return, due to ser being None
        with mock.patch.object(TmcUart, 'read_reg', return_value=
                               b'U\x00o\x03\x05\xffo\xc0\x1e\x00\x00\xca'):
            reg_ans = self.tmc_uart.read_int(0x00)
            self.assertEqual(reg_ans, -1071775744, "read_int is wrong")


if __name__ == '__main__':
    unittest.main()
