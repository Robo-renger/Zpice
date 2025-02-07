#!/usr/bin/env python3

import unittest
from services.Servo180 import Servo180
from mock.PCAMock import PCAMock

class TestServo180(unittest.TestCase):
    def setUp(self):
        self.mock_pwm_driver = PCAMock()
        self.servo = Servo180(9, self.mock_pwm_driver)

    def test_initial_position(self):
        """Test if servo is initialized the center (90 degree)."""
        expected_value = self.mock_pwm_driver.microsecondsToDutycycle(1500)
        self.assertEqual(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            expected_value,
            "Servo should be initialized to 90degrees (1500us)."
        )

    def test_move_forward(self):
        """Test if servo moves forward by increasing the step."""
        initial_value = self.mock_pwm_driver.PCA_channels[9].duty_cycle
        self.servo.setStep(10)
        self.servo.move()
        self.assertGreater(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            initial_value,
            "Servo should increase step when moving forward."
        )
        final_value = self.mock_pwm_driver.microsecondsToDutycycle(1510)
        self.assertEqual(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            final_value,
            "Step of the servo should be increased"
        )

    def test_move_backward(self):
        """Test if servo moves backward by decreasing the step."""
        initial_value = self.mock_pwm_driver.PCA_channels[9].duty_cycle
        self.servo.setStep(-10)
        self.servo.move()
        self.assertLess(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            initial_value,
            "Servo should decrease step when moving backward."
        )
        final_value = self.mock_pwm_driver.microsecondsToDutycycle(1490)
        self.assertEqual(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            final_value,
            "Step of the servo should be decreased"
        )

    def test_set_angle(self):
        """Test if setting an angle results in the correct pulse width."""
        self.servo.setAngle(180)
        expected_value = self.mock_pwm_driver.microsecondsToDutycycle(2000)
        self.assertEqual(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            expected_value,
            "Servo should set pulse width to 2000 µs at 180 degrees."
        )
        
        self.servo.setAngle(0)
        expected_value = self.mock_pwm_driver.microsecondsToDutycycle(1000)
        self.assertEqual(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            expected_value,
            "Servo should set pulse width to 1000 µs at 0 degrees."
        )

    def test_angle_out_of_range(self):
        """Test if setting an invalid angle raises a ValueError."""
        with self.assertRaises(ValueError):
            self.servo.setAngle(200)

if __name__ == '__main__':
    unittest.main()
