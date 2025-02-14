#!/usr/bin/env python3
from zope.interface import implementer
from interface.Servo360Interface import IServo360
from interface.PWMDriver import PWMDriver
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity
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
        self.__forward_value = 1000
        self.__stop_value = 1475
        self.__backward_value = 2000
        self.__delay = 0.0001
    
    def goForward(self) -> None:
        """
        Makes the servo move clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__forward_value)
        # time.sleep(self.__delay)
        # self.Stop()
        # time.sleep(0.01)
        
    def goBackwards(self) -> None:
        """
        Makes the servo move counter clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        self.__pwm_driver.PWMWrite(self.__channel, self.__backward_value)
        # time.sleep(self.__delay)
        # self.Stop()
        # time.sleep(0.01)

    def Stop(self) -> None:
        """
        Makes the servo stop the motion
        :param channel: the channel the servo is connected to.
        """
        try:
            self.__pwm_driver.PWMWrite(self.__channel, self.__stop_value)
        except ValueError as e:
            Logger.logToFile(LogSeverity.ERROR, f"Failed to stop the servo. {e}", "Servo360")
            Logger.logToGUI(LogSeverity.ERROR, f"Failed to stop the servo. {e}", "Servo360")

    def setForward(self, value: int) -> None:
        """
        Set the forward value for the servo.
        param: value: Value to be set.
        """
        self.__forward_value = value
        
    def setStop(self, value: int) -> None:
        """
        Set the stop value for the servo.
        param: value: Value to be set.
        """
        self.__stop_value = value
    
    def setBackward(self, value: int) -> None:
        """
        Set the backward value for the servo.
        param: value: Value to be set.
        """
        self.__backward_value = value
        
    def setDelay(self, value: float) -> None:
        """
        Set the delay value for the servo.
        param: value: Value to be set.
        """
        self.__delay = value

    def getForward(self) -> int:
        """
        Get the forward value for the servo.
        return: forward value.
        """
        return self.__forward_value
    
    def getStop(self) -> int:
        """
        Get the stop value for the servo.
        return: stop value.
        """
        return self.__stop_value
    
    def getBackward(self) -> int:
        """
        Get the backward value for the servo.
        return: backward value.
        """
        return self.__backward_value
    
    def getDelay(self) -> float:
        """
        Get the delay value for the servo.
        return: delay value.
        """
        return self.__delay