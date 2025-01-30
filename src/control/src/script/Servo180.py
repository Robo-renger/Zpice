#!/usr/bin/env python3

from zope.interface import implementer
from interface.Servo180Interface import IServo180
from interface.PWMDriver import PWMDriver

@implementer(IServo180)
class Servo180:
    """
    180Servo class to control the rotation angle of 180servo moves in range of (1000 - 2000 us)
    us = 2000 --> Servo goes to angle 180
    us = 1500 --> Servo goes to angle 90 
    us = 1000 --> Servo goes to angle 0 
    """
    def __init__(self, channel : int, pwm_driver: PWMDriver, max_limit: int = 2000, min_limit: int = 1000) -> None:
        if (min_limit > max_limit):
            raise ValueError("min_limit must be less than max_limit.")
        """
        We shouldnt handle channel issues in component other than the PWM driver
        As if for some reason the PWM driver was changed to have a 20 channel this
        if condition would stil raise an error (Single responsibilty prinicple)
        Reviewed by Ziad
        """
        if not 0 <= channel <= 15:
            raise ValueError("Channel must be between 0 and 15.")
        self.__channel = channel
        self.__pwm_driver = pwm_driver
        self.__max_limit = max_limit
        self.__min_limit = min_limit
        self.__prev_value = 1500
        self.setAngle() # Initialize the servo to angle 90 should be the center forward 


    """"
    We need a way to somehow change the step that it moves with
    Without changing the implementation of the class (e.g setter for the client side)
    Reviewed by Ziad
    """
    def move(self, step: int = 1) -> None:
        """
        Controls the movment of the servo by specific step up or down
        :param: step: Step to move the servo by (positive or negative).
        """
        self.bounded_value = self.keepInBounds(self.__prev_value + step) 
        self.__pwm_driver.PWMWrite(self.__channel, self.bounded_value)
        self.__prev_value = self.bounded_value


    """
    This function need to log/output if it have gone out of range 
    Reviewed by Ziad
    """
    def keepInBounds(self, value: int) -> int:
        """
        Clamp the value within the allowable range.
        """ 
        return max(self.__min_limit, min(value, self.__max_limit))
    
    def setAngle(self, angle: int = 90):
        """
        Moves the servo to a specific angle
        :param: angle: Desired angle
        """
        if not 0 <= angle <= 180:
            raise ValueError("Angle must be between 0 and 180 degrees.")
        pulse_width = int(self.__min_limit + ((angle / 180.0) * (self.__max_limit - self.__min_limit)))
        self.__pwm_driver.PWMWrite(self.__channel, pulse_width)
        self.__prev_value = pulse_width