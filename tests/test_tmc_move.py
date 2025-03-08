"""
test for _tmc_move.py
"""

import time
import unittest
from src.tmc_driver.tmc_2209 import *

class TestTMCMove(unittest.TestCase):
    """TestTMCMove"""

    def setUp(self):
        """setUp"""
        self.tmc = Tmc2209(None, TmcMotionControlStepDir(16, 20))

        # these values are normally set by reading the driver
        self.tmc.mres = 2

        self.tmc.acceleration_fullstep = 100000
        self.tmc.max_speed_fullstep = 10000
        self.tmc.movement_abs_rel = MovementAbsRel.ABSOLUTE

    def tearDown(self):
        """tearDown"""
        self.tmc.set_deinitialize_true()

    def test_run_to_position_steps(self):
        """test_run_to_position_steps"""

        self.tmc.run_to_position_steps(400, MovementAbsRel.RELATIVE)
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_steps(-200, MovementAbsRel.RELATIVE)
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 200, f"actual position: {pos}, expected position: 200")

        self.tmc.run_to_position_steps(400)
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

    def test_run_to_position_other(self):
        """test_run_to_position_other"""
        self.tmc.run_to_position_fullsteps(200)                              #move to position 200 (fullsteps)
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_fullsteps(0)                                #move to position 0
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 0, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_fullsteps(200, MovementAbsRel.RELATIVE)     #move 200 fullsteps forward
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_fullsteps(-200, MovementAbsRel.RELATIVE)    #move 200 fullsteps backward
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 0, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_steps(400)                                  #move to position 400 (µsteps)
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_steps(0)                                    #move to position 0
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 0, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_revolutions(1)                              #move 1 revolution forward
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.run_to_position_revolutions(0)                              #move 1 revolution backward
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 0, f"actual position: {pos}, expected position: 400")

    def test_run_to_position_steps_threaded(self):
        """test_run_to_position_steps_threaded"""
        self.tmc.tmc_mc.run_to_position_steps_threaded(400, MovementAbsRel.RELATIVE)
        self.tmc.tmc_mc.wait_for_movement_finished_threaded()
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.tmc_mc.run_to_position_steps_threaded(-200, MovementAbsRel.RELATIVE)
        self.tmc.tmc_mc.wait_for_movement_finished_threaded()
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 200, f"actual position: {pos}, expected position: 200")

        self.tmc.tmc_mc.run_to_position_steps_threaded(400)
        self.tmc.tmc_mc.wait_for_movement_finished_threaded()
        pos = self.tmc.tmc_mc.current_pos
        self.assertEqual(pos, 400, f"actual position: {pos}, expected position: 400")

        self.tmc.tmc_mc.run_to_position_steps_threaded(800)
        time.sleep(0.05)
        self.tmc.tmc_mc.stop()
        pos = self.tmc.tmc_mc.current_pos
        print(f"motorposition: {pos}")
        self.assertTrue(400 < pos < 800, f"actual position: {pos}, expected position: 400 < pos < 800")

if __name__ == '__main__':
    unittest.main()
