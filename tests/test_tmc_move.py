#pylint: disable=missing-function-docstring
#pylint: disable=missing-class-docstring
#pylint: disable=wildcard-import
#pylint: disable=pointless-statement
#pylint: disable=unused-wildcard-import
#pylint: disable=bare-except
#pylint: disable=protected-access
"""
test for _tmc_move.py
"""

import unittest
from src.tmc_driver.tmc_2209 import *

class TestTMCMove(unittest.TestCase):
    """TestTMCMove"""

    def setUp(self):
        """setUp"""
        self.tmc = Tmc2209(21, 16, 20, serialport=None, skip_uart_init=True)

        # these values are normally set by reading the driver
        self.tmc.mres = 2

        self.tmc.acceleration_fullstep = 100000
        self.tmc.max_speed_fullstep = 10000
        self.tmc.movement_abs_rel = MovementAbsRel.ABSOLUTE

    def tearDown(self):
        """tearDown"""
        self.tmc.set_deinitialize_true

    def test_run_to_position_steps(self):
        """test_run_to_position_steps"""

        self.tmc.run_to_position_steps(400, MovementAbsRel.RELATIVE)
        pos = self.tmc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_steps(-200, MovementAbsRel.RELATIVE)
        pos = self.tmc.current_pos
        self.assertEqual(pos, 200, f"actual position: {pos}, expected position: 200")

        self.tmc.run_to_position_steps(400)
        pos = self.tmc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

    def test_run_to_position_steps_threaded(self):
        """test_run_to_position_steps_threaded"""
        self.tmc.run_to_position_steps_threaded(400, MovementAbsRel.RELATIVE)
        self.tmc.wait_for_movement_finished_threaded()
        pos = self.tmc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_steps_threaded(-200, MovementAbsRel.RELATIVE)
        self.tmc.wait_for_movement_finished_threaded()
        pos = self.tmc.current_pos
        self.assertEqual(pos, 200, f"actual position: {pos}, expected position: 200")

        self.tmc.run_to_position_steps_threaded(400)
        self.tmc.wait_for_movement_finished_threaded()
        pos = self.tmc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_steps_threaded(800)
        time.sleep(0.05)
        self.tmc.stop()
        pos = self.tmc.current_pos
        print(f"motorposition: {pos}")
        self.assertTrue(400 < pos < 800, f"actual position: {pos}, expected position: 400 < pos < 800")

if __name__ == '__main__':
    unittest.main()
