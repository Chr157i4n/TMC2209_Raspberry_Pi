#pylint: disable=invalid-name
#pylint: disable=missing-function-docstring
#pylint: disable=missing-class-docstring
#pylint: disable=wildcard-import
#pylint: disable=pointless-statement
#pylint: disable=unused-wildcard-import
"""
test for TMC_2209_uart.py
"""

import unittest
import mock
from src.TMC_2209._TMC_2209_logger import *
from src.TMC_2209._TMC_2209_uart import *


class TestTMCUart(unittest.TestCase):
    """TestTMCUart"""

    def setUp(self):
        """setUp"""
        self.tmc_logger = TMC_logger()
        self.tmc_uart = TMC_UART(self.tmc_logger, serialport=None, baudrate=115200)

    def test_read_int(self):
        """test_read_int"""
        with mock.patch.object(TMC_UART, 'read_reg', return_value=
                               b'U\x00o\x03\x05\xffo\xc0\x1e\x00\x00\xca'):
            reg_ans = self.tmc_uart.read_int(0x00)
            self.assertEqual(reg_ans, -1071775744, "read_int is wrong")


if __name__ == '__main__':
    unittest.main()
