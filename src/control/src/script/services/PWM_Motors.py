#!/usr/bin/env python3
from zope.interface import implementer
from interface.iPWM_Motors import iPWM_Motors
import time
from interface.iLoggable import iLoggable
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from helpers.JsonFileHandler import JsonFileHandler
from script.LogPublisherNode import LogPublisherNode
from utils.Configurator import Configurator
@implementer(iPWM_Motors, iLoggable)
class PWM_Motors:
    def __init__(self, pca, channel, min_value, max_val, init_value = 1500):
        self.pca = pca
        self.channel = channel
        self.min_value = min_value
        self.max_val = max_val
        self.current_value = init_value
        self.__smoothing = 0
        self.__smoother = None
        self.__setSmoother()
        self.json_file_handler = JsonFileHandler()
        self.log_publisher = LogPublisherNode()
        
        # self.stop() # Initialize the motor by 1500 value
        # print("ana nayem")
        # time.sleep(3)
    def __setSmoother(self):
        smootherType = Configurator().fetchData(Configurator.CHANGEABLE_MODULES)['SMOOTHING_STRAT']
        if smootherType == "EXPONENTIAL":
            from smoothing_strategies.ExponentialSmoothing import ExponentialSmoothing
            self.__smoother = ExponentialSmoothing()
            print("expooooo")
        else:
            from smoothing_strategies.DefaultSmoothing import DefaultSmoothing
            self.__smoother = DefaultSmoothing()
    def output_raw(self, value: int) -> None:
        """
        Publish a raw PWM signal to the motor controller without smoothing.

        Raises:
            ValueError: If the value is out of bounds.
        """
        if value < self.min_value or value > self.max_val:
            self.logToFile(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_val}.", "PWM_Motors")
            self.logToGUI(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_val}.", "PWM_Motors")
            raise ValueError(f"Value must be between {self.min_value} and {self.max_val}.")
        self.pca.PWMWrite(self.channel, value)

    def drive(self, value: int, en_smoothing: bool = True) -> None:
        """
        Drive the motor with a PWM signal.

        Raises:
            ValueError: If the value is out of bounds.
        """
        if value < self.min_value or value > self.max_val:
            self.logToFile(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_val}.", "PWM_Motors")
            self.logToGUI(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_val}.", "PWM_Motors")
            raise ValueError(f"Value must be between {self.min_value} and {self.max_val}.")

        value = self._ensure_bounds(value)

        if en_smoothing:
            if self.channel == 5:
                value = self._smoothing(value)
            
            if self.channel == 5:
                print(f"[Smoothing Enabled] Channel {self.channel} - Smoothed Value: {value}")
        else:
            if self.channel == 5:
                print(f"[Smoothing Disabled] Channel {self.channel} - Raw Value: {value}")

        # Write to PCA regardless of smoothing
        self.pca.PWMWrite(self.channel, value)

        # Update current_value regardless of smoothing to keep state consistent
        self.current_value = value

        
    def _smoothing(self, value: int) -> None:
        """
        Smooth the PWM signal to the motor controller.
        Used by drive method.
        """
        smoothed_value  = self.__smoother.smooth(self.current_value,value)      
        return smoothed_value 
    
    def stop(self) -> None:
        """
        Stop the motor.
        """
        self.pca.PWMWrite(self.channel, 1500)
        
    def _ensure_bounds(self, value: int) -> int:
        """
        Ensure the value is within the bounds of min_value and max_val.
        """
        if value < self.min_value:
            return self.min_value
        elif value > self.max_val:
            return self.max_val
        return value
    
    def logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log)
        return log
    
    def logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name)
        return log

