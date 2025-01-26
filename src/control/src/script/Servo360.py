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
        if not 0 <= channel <= 15:
            raise ValueError("Channel must be between 0 and 15.")
        self.__pwm_driver = pwm_driver
        self.__channel = channel
        self.__forward_value = 1495
        self.__stop_value = 1500
        self.__backward_value = 1505  

    def goForward(self, delay: int = 0.0001) -> None:
        """
        Makes the servo move clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__forward_value)
        time.sleep(delay)
        self.Stop()
        

    def goBackwards(self, delay: int = 0.0001) -> None:
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
    