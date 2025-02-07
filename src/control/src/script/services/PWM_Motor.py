#!/usr/bin/env python3
from zope.interface import implementer
from interface.iPWM_Motors import iPWM_Motors
from smoothing_strategies.DefaultSmoothing import DefaultSmoothing
import time
@implementer(iPWM_Motors)
class PWMMotor:
    def __init__(self, pca, channel, min_value, max_val, init_value = 1500, smoothing_strategy=None):
        self.pca = pca
        self.channel = channel
        self.min_value = min_value
        self.max_val = max_val
        self.current_value = init_value
        self.__smoothing = 0
        
        self.smoothing_strategy = smoothing_strategy or DefaultSmoothing()
        
        self.stop() # Initialize the motor by 1500 value

    def output_raw(self, value: int) -> None:
        """
        Publish a raw PWM signal to the motor controller without smoothing.
        """
        if value < self.min_value or value > self.max_val:
            raise ValueError(f"Value must be between {self.min_value} and {self.max_val}.")
        self.pca.PWMWrite(self.channel, value)
        self.current_value = value

    def drive(self, value: int, en_smoothing: bool = True) -> None:

        if value < self.min_value or value > self.max_val:
            raise ValueError(f"Value must be between {self.min_value} and {self.max_val}.")
        
        value = self._ensure_bounds(value) 
        
        if en_smoothing:
            self.current_value = self.smoothing_strategy.smooth(self.current_value, value)
            self.pca.PWMWrite(self.channel, self.current_value)
            time.sleep(0.1)
        else:
            self.pca.PWMWrite(self.channel, value)
            self.current_value = value
        
        
        
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

