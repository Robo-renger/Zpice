#!/usr/bin/env python3
from zope.interface import implementer
from interface.Servo360Interface import IServo360
from interface.PWMDriver import PWMDriver
from interface.iLoggable import iLoggable
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from helpers.JsonFileHandler import JsonFileHandler
from nodes.LogPublisherNode import LogPublisherNode
import time
@implementer(IServo360, iLoggable)
class Servo360:
    """
    360Servo class to control the rotation angle of 360servo moves in range of (1000 - 2000 us)
    ms > 1500 --> counter clockwise 
    ms < 1500 --> clockwise 
    ms = 1500 stop 
    """
    def __init__(self, channel: int, pwm_driver: PWMDriver):
        self.json_file_handler = JsonFileHandler()
        self.log_publisher = LogPublisherNode()
        self.__pwm_driver = pwm_driver
        self.__channel = channel
        self.__forward_value = 1495
        self.__stop_value = 1500
        self.__backward_value = 1505
    
    def goForward(self) -> None:
        """
        Makes the servo move clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        try:
            self.__pwm_driver.PWMWrite(self.__channel, self.__forward_value)
            time.sleep(self.__delay)
            self.Stop()
        except ValueError as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to move the servo forward. {e}", "Servo360")
            self.logToGUI(LogSeverity.ERROR, f"Failed to move the servo forward. {e}", "Servo360")
        
    def goBackwards(self) -> None:
        """
        Makes the servo move counter clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
        try:
            self.__pwm_driver.PWMWrite(self.__channel, self.__backward_value)
            time.sleep(self.__delay)
            self.Stop()
        except ValueError as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to move the servo backwards. {e}", "Servo360")
            self.logToGUI(LogSeverity.ERROR, f"Failed to move the servo backwards. {e}", "Servo360")

    def Stop(self) -> None:
        """
        Makes the servo stop the motion
        :param channel: the channel the servo is connected to.
        """
        try:
            self.__pwm_driver.PWMWrite(self.__channel, self.__stop_value)
        except ValueError as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to stop the servo. {e}", "Servo360")
            self.logToGUI(LogSeverity.ERROR, f"Failed to stop the servo. {e}", "Servo360")

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
    
    def logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log)
        return log
    
    def logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name)
        return log