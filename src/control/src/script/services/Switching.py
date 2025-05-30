#!/usr/bin/env python3
from zope.interface import implementer
from interface.iSwitching import iSwitching
import RPi.GPIO as GPIO
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity

@implementer(iSwitching)
class Switching:
    def __init__(self, pin):
        
        """
        Initialize the Switching object.
        With specified pin.
        """      
        self.gpio = GPIO
        self.gpio.setmode(GPIO.BCM)
        self.gpio.setup(pin, GPIO.OUT)
        self.gpio.output(pin, GPIO.LOW) # Initialize the pin to low
        self.pin = pin
        self.opened = False
        
    def open(self) -> None:
        self.gpio.output(self.pin, GPIO.HIGH)
        self.opened = True
        Logger.logToFile(LogSeverity.INFO, "Switching open.", "Switching")
        Logger.logToGUI(LogSeverity.INFO, "Switching open.", "Switching")
        
    def close(self) -> None:
        self.gpio.output(self.pin, GPIO.LOW)
        self.opened = False
        Logger.logToFile(LogSeverity.INFO, "Switching close.", "Switching")
        Logger.logToGUI(LogSeverity.INFO, "Switching close.", "Switching")
        
    def toggle(self) -> None:
        if self.opened:
            self.close()
            Logger.logToFile(LogSeverity.INFO, "Switching close.", "Switching")
            Logger.logToGUI(LogSeverity.INFO, "Switching close.", "Switching")
        else:
            self.open()
            Logger.logToFile(LogSeverity.INFO, "Switching open.", "Switching")
            Logger.logToGUI(LogSeverity.INFO, "Switching open.", "Switching")
            
    def is_open(self) -> bool:
        return self.opened
    
    def is_closed(self) -> bool:
        return not self.opened