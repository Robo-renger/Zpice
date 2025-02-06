#!/usr/bin/env python3

from script.Servo360 import Servo360
from control.src.script.services.PCADriver import PCA
from script.interface.PWMDriver import PWMDriver
import time

class Servo360HardwareTest:
    def __init__(self, channel: int, pwm_driver: PWMDriver):
        self.servo360 = Servo360(channel, pwm_driver) 

    def test_go_forward(self):
        while True:
            delay = input("Enter delay in seconds (or 'c' to change motion): ").strip()
            if delay.lower() == 'c':
                break
            else:
                delay = float(delay)
                self.servo360.setValues(delay=delay)
                self.servo360.goForward()

    def test_go_backward(self):
        while True:
            delay = input("Enter delay in seconds (or 'c' to change motion): ").strip()
            if delay.lower() == 'c':
                break
            else:
                delay = float(delay)
                self.servo360.setValues(delay=delay)
                self.servo360.goBackwards()

    def test_stop(self):
        self.servo360.Stop()


if __name__ == '__main__':
    try:
        pwm_driver = PCA.getInst()
        tester = Servo360HardwareTest(9, pwm_driver)
        while True:
            motion = input("\nEnter the motion (f -> forward, b -> backward, s -> stop, q -> quit): ").strip().lower()      
            if motion == 'f':
                tester.test_go_forward()         
            elif motion == 'b':
                tester.test_go_backward()
            elif motion == 's':
                tester.test_stop()
            elif motion == 'q':
                print("Exiting...")
                break
            else:
                print("Invalid input. Please enter 'f', 'b', 's', or 'q'.")
    except KeyboardInterrupt:
        print("\nExiting...")

