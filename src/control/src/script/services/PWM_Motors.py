#!/usr/bin/env python3
from zope.interface import implementer
from interface.iPWM_Motors import iPWM_Motors
import time
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity
from utils.Configurator import Configurator
import threading

@implementer(iPWM_Motors)
class PWM_Motors:
    def __init__(self, pca, channel, min_value, max_val, init_value = 1500):
        self.pca = pca
        self.channel = channel
        self.min_value = min_value
        self.max_val = max_val
        self.current_value = init_value
        self.__smoother = None
        self.__setSmoother()
        
        # Start motor initialization in a new thread
        init_thread = threading.Thread(target=self.__initialize_motor)
        init_thread.daemon = True
        init_thread.start()
    
    
    def __initialize_motor(self):
        self.stop()
        time.sleep(3)
        
    def setSpeed(self, min:int, max:int):
        self.min_value = min    
        self.max_value = max    
    def __setSmoother(self):
        smootherType = Configurator().fetchData(Configurator.CHANGEABLE_MODULES)['SMOOTHING_STRAT']
        if smootherType == "EXPONENTIAL":
            from smoothing_strategies.ExponentialSmoothing import ExponentialSmoothing
            self.__smoother = ExponentialSmoothing()
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
            Logger.logToFile(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_val}.", "PWM_Motors")
            Logger.logToGUI(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_val}.", "PWM_Motors")
            raise ValueError(f"Value must be between {self.min_value} and {self.max_val}.")
        self.pca.PWMWrite(self.channel, value)

    def drive(self, value: int, en_smoothing: bool = True) -> None:
        """
        Drive the motor with a PWM signal.

        Raises:
            ValueError: If the value is out of bounds.
        """
        if value < self.min_value or value > self.max_val:
            Logger.logToFile(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_val}.", "PWM_Motors")
            Logger.logToGUI(LogSeverity.ERROR, f"Value must be between {self.min_value} and {self.max_val}.", "PWM_Motors")
            raise ValueError(f"Value must be between {self.min_value} and {self.max_val}.")

        value = self._ensure_bounds(value)

        if en_smoothing:
            value = self._smoothing(value)
            # print(f"writing: {value}")
            self.pca.PWMWrite(self.channel,value)
        else:
            self.pca.PWMWrite(self.channel, value)
            # print(value)
        self.current_value = value
    def _smoothing(self, value: int) -> None:
        """
        Smooth the PWM signal to the motor controller.
        Used by drive method.
        """
        return self.__smoother.smooth(self.current_value,value)
        
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
