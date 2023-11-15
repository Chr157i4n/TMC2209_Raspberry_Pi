#pylint: disable=invalid-name
#pylint: disable=missing-function-docstring
#pylint: disable=missing-class-docstring
"""
test for TMC_2209_move.py
"""

import unittest
from src.TMC_2209.TMC_2209_StepperDriver import *

class TestTMCMove(unittest.TestCase):

    def setUp(self):
        self.tmc = TMC_2209(21, 16, 20, skip_uart_init=True)
        self.tmc.set_acceleration_fullstep(100000)
        self.tmc.set_max_speed_fullstep(10000)
        self.tmc.set_movement_abs_rel(MovementAbsRel.ABSOLUTE)

    def tearDown(self):
        self.tmc.set_deinitialize_true

    def test_run_to_position_steps(self):

        self.tmc.run_to_position_steps(400, MovementAbsRel.RELATIVE)
        pos = self.tmc.get_current_position()
        self.assertEqual(pos, 400, "run_to_position_steps is wrong")

        self.tmc.run_to_position_steps(-200, MovementAbsRel.RELATIVE)
        pos = self.tmc.get_current_position()
        self.assertEqual(pos, 200, "run_to_position_steps is wrong")

        self.tmc.run_to_position_steps(400)
        pos = self.tmc.get_current_position()
        self.assertEqual(pos, 400, "run_to_position_steps is wrong")

    def test_run_to_position_steps_threaded(self):
        self.tmc.run_to_position_steps_threaded(400, MovementAbsRel.RELATIVE)
        self.tmc.wait_for_movement_finished_threaded()
        pos = self.tmc.get_current_position()
        self.assertEqual(pos, 400, "run_to_position_steps_threaded is wrong")

        self.tmc.run_to_position_steps_threaded(-200, MovementAbsRel.RELATIVE)
        self.tmc.wait_for_movement_finished_threaded()
        pos = self.tmc.get_current_position()
        self.assertEqual(pos, 200, "run_to_position_steps_threaded is wrong")

        self.tmc.run_to_position_steps_threaded(400)
        self.tmc.wait_for_movement_finished_threaded()
        pos = self.tmc.get_current_position()
        self.assertEqual(pos, 400, "run_to_position_steps_threaded is wrong")

if __name__ == '__main__':
    unittest.main()
