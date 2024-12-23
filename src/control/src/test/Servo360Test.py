#!/usr/bin/env python3

from script.Servo360 import Servo360
from script.services.PCADriver import PCA
import rospy
import time


class TestServo360():

    def __init__(self) -> None:
        # rospy.init_node('test_servo_360', anonymous=True)
        self.servo = Servo360(9, PCA.getInst())
    
    def testForward(self) -> None:
        forward_value = self.servo._Servo360__pwm_driver.getLastWrittenValue(3)
        self.assertEqual(forward_value, 1495, "Forward PWM value is incorrect.")

        # After the slight delay, ensure the stop value is written
        time.sleep(0.001)  # Wait for the stop value to be written
        stop_value = self.servo._Servo360__pwm_driver.getLastWrittenValue(3)
        self.assertEqual(stop_value, 1500, "Stop PWM value is incorrect.")
    def testBackward(self) -> None:
        assert self.servo.goBackwards()

    def testStop(self) -> None:
        assert self.servo.Stop()


if __name__ == "__main__":
    test = TestServo360()
    test.testForward()
