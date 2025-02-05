#!/usr/bin/env python3
import logging
from zope.interface import implementer
from interface.Servo180Interface import IServo180
from interface.PWMDriver import PWMDriver
from interface.iLoggable import iLoggable
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from helpers.JsonFileHandler import JsonFileHandler
from LogPublisherNode import LogPublisherNode
logging.basicConfig(level=logging.INFO)

@implementer(IServo180, iLoggable)
class Servo180:
    """
    180Servo class to control the rotation angle of 180servo moves in range of (1000 - 2000 us)
    us = 2000 --> Servo goes to angle 180
    us = 1500 --> Servo goes to angle 90 
    us = 1000 --> Servo goes to angle 0 
    """
    def __init__(self, channel : int, pwm_driver: PWMDriver, max_limit: int = 2000, min_limit: int = 1000) -> None:
        """
        Initialize the servo.
        raises:
            ValueError: If min_limit is greater than max_limit.
        """
        self.json_file_handler = JsonFileHandler()
        self.log_publisher = LogPublisherNode()
        if (min_limit > max_limit):
            self.logToFile(LogSeverity.ERROR, "min_limit must be less than max_limit.", "Servo180")
            self.logToGUI(LogSeverity.ERROR, "min_limit must be less than max_limit.", "Servo180")
            raise ValueError("min_limit must be less than max_limit.")
        self.__channel = channel
        self.__pwm_driver = pwm_driver
        self.__max_limit = max_limit
        self.__min_limit = min_limit
        self.__prev_value = 1500
        self.__step = 1
        self.setAngle() # Initialize the servo to angle 90 should be the center forward 

    def setStep(self, step: int) -> None:
        """
        Set the step value for the servo.
        param: step: Step to move the servo by (positive or negative).
        """
        self.__step = step

    def getStep(self) -> int:
        """
        Get the step value for the servo.
        """
        return self.__step

    def move(self) -> None:
        """
        Controls the movment of the servo by specific step up or down 
        """
        self.bounded_value = self._keepInBounds(self.__prev_value + self.__step) 
        self.__pwm_driver.PWMWrite(self.__channel, self.bounded_value)
        self.__prev_value = self.bounded_value

    def _keepInBounds(self, value: int) -> int:
        """
        Clamp the value within the allowable range.
        """ 
        if value < self.__min_limit:
            logging.warning("Servo value is below 1000us")
        elif value > self.__max_limit:
            logging.warning("Servo value is above 2000us")
        return max(self.__min_limit, min(value, self.__max_limit))
    
    def setAngle(self, angle: int = 90):
        """
        Moves the servo to a specific angle
        :param: angle: Desired angle
        """
        if not 0 <= angle <= 180:
            self.logToFile(LogSeverity.ERROR, "Angle must be between 0 and 180 degrees.", "Servo180")
            self.logToGUI(LogSeverity.ERROR, "Angle must be between 0 and 180 degrees.", "Servo180")
            raise ValueError("Angle must be between 0 and 180 degrees.")
        pulse_width = int(self.__min_limit + ((angle / 180.0) * (self.__max_limit - self.__min_limit)))
        try:
            self.__pwm_driver.PWMWrite(self.__channel, pulse_width)
            self.__prev_value = pulse_width
        except ValueError as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to set the angle. {e}", "Servo180")
            self.logToGUI(LogSeverity.ERROR, f"Failed to set the angle. {e}", "Servo180")

    def logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log.toDictionary())
        return log
    
    def logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name)
        return log