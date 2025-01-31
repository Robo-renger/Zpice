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
        self.__pwm_driver = pwm_driver
        self.__channel = channel
        self.setValues()
    
    def goForward(self) -> None:
        """
        Makes the servo move clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__forward_value)
        time.sleep(self.__delay)
        self.Stop()
        
    def goBackwards(self) -> None:
        """
        Makes the servo move counter clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__backward_value)
        time.sleep(self.__delay)
        self.Stop()

    def Stop(self) -> None:
        """
        Makes the servo stop the motion
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__stop_value)

    def setValues(self, forward_value: int = 1495 , stop_value: int = 1500, backward_value: int = 1505, delay: float = 0.001) -> None:
        """
        Set the values for the servo.
        param: forward_value: Forward value to be set.
        param: stop_value: Stop value to be set.
        param: backward_value: Backward value to be set.
        param: delay: Delay value to be set.
        """
        self.__forward_value = forward_value
        self.__stop_value = stop_value
        self.__backward_value = backward_value
        self.__delay = delay

    def getValues(self) -> tuple:
        """
        Get the values for the servo.
        """
        return (self.__forward_value, self.__stop_value, self.__backward_value, self.__delay)
