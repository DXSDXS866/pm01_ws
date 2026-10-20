import unittest

from pm01_aruco_detection.pm01_gamepad_mapping import shape_stick


class Pm01GamepadMappingTest(unittest.TestCase):
    def test_shape_stick_zero_inside_deadband(self):
        self.assertEqual(shape_stick(0.01, 0.4, 0.02, 0.35), 0.0)

    def test_shape_stick_applies_minimum_positive_stick(self):
        self.assertAlmostEqual(shape_stick(0.05, 0.4, 0.01, 0.35), 0.35)

    def test_shape_stick_applies_minimum_negative_stick(self):
        self.assertAlmostEqual(shape_stick(-0.05, 0.4, 0.01, 0.35), -0.35)

    def test_shape_stick_clamps_to_one(self):
        self.assertAlmostEqual(shape_stick(2.0, 0.4, 0.01, 0.35), 1.0)


if __name__ == "__main__":
    unittest.main()
