"""
test for _tmc_com_spi.py
"""

import unittest
from unittest import  mock
from src.tmc_driver._tmc_logger import *
from src.tmc_driver.com._tmc_com_spi import *


class TestTmcComSpi(unittest.TestCase):
    """TestTmcComSpi"""

    def setUp(self):
        """setUp"""
        self.tmc_logger = TmcLogger()
        self.tmc_uart = TmcComSpi(None, 115200, 0, self.tmc_logger)

    def test_read_int(self):
        """test_read_int"""
        with mock.patch.object(TmcComSpi, 'read_reg', return_value=
                               b'U\xc0\x1e\x00\x00\xca'):
            reg_ans = self.tmc_uart.read_int(0x00)
            self.assertEqual(reg_ans, 94283625398474, "read_int is wrong")


if __name__ == '__main__':
    unittest.main()
