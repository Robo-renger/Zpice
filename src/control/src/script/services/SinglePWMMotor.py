#!/usr/bin/env python3
from zope.interface import implementer
from interface.iDCMotor import iDCMotor
import RPi.GPIO as GPIO
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity
import time

@implementer(iDCMotor)
class SinglePWMDCMotor:
    def __init__(self, pca, channel, dir_pin, min_value=350, max_value=19500):
        self.pca = pca
        self.channel = channel
        self.dir_pin = dir_pin
        self.min_value = min_value
        self.max_value = max_value 
        self.backwardPWM = max_value
        self.forwardPWM = min_value
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)  

        self.stop()

    def driveForward(self) -> None:
        """
        Set the speed of the motor using a PWM signal.
        Speed is given as an 8-bit value (0-255).
        """
        self.drive(self.forwardPWM)
        self.direction('f')

        
    def driveBackward(self) -> None:
        """
        Set the speed of the motor using a PWM signal.
        Speed is given as an 8-bit value (0-255).
        """
        self.drive(self.backwardPWM)
        self.direction('r')


    def drive(self, speed: int) -> None:
        """
        Set speed based on the current direction.
        """
        if speed < self.min_value or speed > self.max_value:
            Logger.logToFile(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_value}.", "SinglePWMMotor")
            Logger.logToGUI(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_value}.", "SinglePWMMotor")
            raise ValueError(f"Value must be between {self.min_value} and {self.max_value}.")
        
        self.pca.PWMWrite(self.channel, speed)
        

    def direction(self, direction: str) -> None:
        """
        Set Direction for DC Motor
        Parameters: direction --> 'f' for forward, 'r' for reverse
        """
        if direction not in ["f", "r"]:
            Logger.logToFile(LogSeverity.ERROR, "Invalid direction. Use 'f' for forward or 'r' for reverse.", "SinglePWMMotor")
            Logger.logToGUI(LogSeverity.ERROR, "Invalid direction. Use 'f' for forward or 'r' for reverse.", "SinglePWMMotor")
            raise ValueError("Invalid direction. Use 'f' for forward or 'r' for reverse.")

        if direction == "f":
            GPIO.output(self.dir_pin, GPIO.HIGH)
        elif direction == "r":
            GPIO.output(self.dir_pin, GPIO.LOW)
        else:
            self.stop()


    def stop(self) -> None:
        """
        Stop the motor.
        """
        self.pca.PWMWrite(self.channel, 350)
        GPIO.output(self.dir_pin, GPIO.LOW)

    def setPWM(self, backPWM : int, forwardPWM: int):
        if forwardPWM < self.min_value or backPWM > self.max_value:
            Logger.logToFile(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_value}.", "SinglePWMMotor")
            Logger.logToGUI(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_value}.", "SinglePWMMotor")
            raise ValueError(f"Value must be between {self.min_value} and {self.max_value}.")
        self.backwardPWM = backPWM
        self.forwardPWM = forwardPWM