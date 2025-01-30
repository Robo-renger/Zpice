#!/usr/bin/env python3

import unittest
from script.Servo360 import Servo360
from mock.PCAMock import PCAMock  


class TestServo360(unittest.TestCase):
    def setUp(self):
        self.mock_pwm_driver = PCAMock()
        self.servo = Servo360(9, self.mock_pwm_driver)

    def test_channel_out_of_range(self):
        # Test that an error is raised when the channel is out of range.
        with self.assertRaises(ValueError, msg="Channel must be between 0 and 15."):
            Servo360(20, self.mock_pwm_driver)

    def test_go_forward(self):
        # Test that the servo moves forward with the correct value, however the delay in Servo needs to be commented out.
        self.servo.goForward()
        expected_value = self.mock_pwm_driver.microsecondsToDutycycle(1495)
        # print(f"expected_value of forward = {expected_value}")
        self.assertEqual(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            expected_value,
            "goForward should set the correct duty cycle."
        )

    def test_go_backward(self):
        # Test that the servo moves backward with the correct value, however the delay in Servo needs to be commented out.
        self.servo.goBackwards()
        expected_value = self.mock_pwm_driver.microsecondsToDutycycle(1505)
        # print(f"expected_value of backward = {expected_value}")
        self.assertEqual(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            expected_value,
            "goBackwards should set the correct duty cycle."
        )

    def test_stop(self):
        self.servo.Stop()
        expected_value = self.mock_pwm_driver.microsecondsToDutycycle(1500)
        # print(f"expected_value of stop = {expected_value}")
        self.assertEqual(
            self.mock_pwm_driver.PCA_channels[9].duty_cycle,
            expected_value,
            "Stop should set the correct duty cycle."
        )


if __name__ == '__main__':
    unittest.main()
