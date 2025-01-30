#!/usr/bin/env python3

from script.Servo180 import Servo180
from script.services.PCADriver import PCA
from interface.PWMDriver import PWMDriver
import time

class Servo180HardwareTest:
    def __init__(self, channel: int, pwm_driver: PWMDriver):
        self.servo360 = Servo180(channel, pwm_driver)

    def test(self, step: int):
        self.servo360.move(step)
        time.sleep(1)

if __name__ == '__main__':
    try:
        pwm_driver = PCA.getInst()
        tester = Servo180HardwareTest(9, pwm_driver)
        while True:
            step = input("\nEnter the step")    
            tester.test(step)
    except KeyboardInterrupt:
        print("\nExiting...")

