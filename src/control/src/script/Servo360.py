#!/usr/bin/env python3

from zope.interface import implementer
from interface.Servo360Interface import IServo360
from interface.PWMDriver import PWMDriver
import time
@implementer(IServo360)
class Servo360:
    """
    360Servo class to control the rotation angle of 360servo moves in range of (1000 - 2000 us)
    ms > 1500 --> counter clockwise 
    ms < 1500 --> clockwise 
    ms = 1500 stop 
    """
    def __init__(self, channel: int, pwm_driver: PWMDriver):
        """
        We shouldnt handle channel issues in component other than the PWM driver
        As if for some reason the PWM driver was changed to have a 20 channel this
        if condition would stil raise an error (Single responsibilty prinicple)
        Reviewed by Ziad
        """
        if not 0 <= channel <= 15:
            raise ValueError("Channel must be between 0 and 15.")
        self.__pwm_driver = pwm_driver
        self.__channel = channel
        self.__forward_value = self.__pwm_driver.microsecondsToDutycycle(1495) 
        self.__stop_value = self.__pwm_driver.microsecondsToDutycycle(1500)
        self.__backward_value = self.__pwm_driver.microsecondsToDutycycle(1505)  
    
    
    
    """"
    We need a way to somehow change the step that it moves with
    Without changing the implementation of the class (e.g setter for the client side)
    Reviewed by Ziad
    """
    def goForward(self) -> None:
        """
        Makes the servo move clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__forward_value)
        time.sleep(delay)
        self.Stop()
        
    """"
    We need a way to somehow change the step that it moves with
    Without changing the implementation of the class (e.g setter for the client side)
    Reviewed by Ziad
    """
    def goBackwards(self) -> None:
        """
        Makes the servo move counter clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__backward_value)
        time.sleep(delay)
        self.Stop()

    def Stop(self) -> None:
        """
        Makes the servo stop the motion
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__stop_value)
    