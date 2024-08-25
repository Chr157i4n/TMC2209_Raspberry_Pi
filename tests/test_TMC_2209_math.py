#pylint: disable=invalid-name
#pylint: disable=missing-function-docstring
#pylint: disable=missing-class-docstring
"""
test for TMC_2209_math.py
"""

import unittest
import src.TMC_2209._TMC_2209_math as tmc_math

class TestTMCMath(unittest.TestCase):
    """TestTMCMath"""

    def test_rps_to_vactual(self):
        """test_rps_to_vactual"""
        self.assertEqual(tmc_math.rps_to_vactual(1,400), 559, "rps_to_vactual is wrong")

    def test_vactual_to_rps(self):
        """test_vactual_to_rps"""
        self.assertEqual(round(tmc_math.vactual_to_rps(559,400)), 1, "vactual_to_rps is wrong")

    def test_rps_to_steps(self):
        """test_rps_to_steps"""
        self.assertEqual(round(tmc_math.rps_to_steps(1,400)), 400, "rps_to_steps is wrong")

    def test_steps_to_rps(self):
        """test_steps_to_rps"""
        self.assertEqual(round(tmc_math.steps_to_rps(400,400)), 1, "steps_to_rps is wrong")

    def test_rps_to_tstep(self):
        """test_rps_to_tstep"""
        self.assertEqual(round(tmc_math.rps_to_tstep(1,400, 2)), 234, "rps_to_tstep is wrong")

    def test_steps_to_tstep(self):
        """test_steps_to_tstep"""
        self.assertEqual(round(tmc_math.steps_to_tstep(400,2)), 234, "steps_to_tstep is wrong")

    def test_constrain(self):
        """test_constrain"""
        self.assertEqual(tmc_math.constrain(5,0,10), 5, "test_constrain is wrong")
        self.assertEqual(tmc_math.constrain(15,0,10), 10, "test_constrain is wrong")
        self.assertEqual(tmc_math.constrain(-5,0,10), 0, "test_constrain is wrong")
        self.assertEqual(tmc_math.constrain(-5,-10,10), -5, "test_constrain is wrong")


if __name__ == '__main__':
    unittest.main()
