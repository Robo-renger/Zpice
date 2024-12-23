#!/usr/bin/env python3

from zope.interface import implementer
from interface.Servo360Interface import IServo360
from interface.PWMDriver import PWMDriver
from script.services.PCADriver import PCA
import time
@implementer(IServo360)
class Servo360:
    """
    360Servo class to control the rotation angle of 360servo moves in range of (1 - 2 ms)
    ms > 1.5 --> counter clockwise 
    ms < 1.5 --> clockwise 
    ms = 1.5 stop 
    """
    def __init__(self, channel: int, pwm_driver: PWMDriver):
        self.__pwm_driver = pwm_driver
        if not 0 <= channel <= 15:
            raise ValueError("Channel must be between 0 and 15.")
        self.__channel = channel
        self.__forward_value = self.__pwm_driver.microsecondsToDutycycle(1495) 
        self.__stop_value = self.__pwm_driver.microsecondsToDutycycle(1500)
        self.__backward_value = self.__pwm_driver.microsecondsToDutycycle(1505)  

    def goForward(self) -> None:
        """
        Makes the servo move clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__forward_value)
        time.sleep(0.0001)
        self.Stop()
        

    def goBackwards(self) -> None:
        """
        Makes the servo move counter clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__backward_value)
        time.sleep(0.0001)
        self.Stop()

    def Stop(self) -> None:
        """
        Makes the servo stop the motion
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__stop_value)
    