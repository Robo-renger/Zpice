from zope.interface import implementer
from interface.iSwitching import iSwitching
import RPi.GPIO as GPIO
from interface.iLoggable import iLoggable
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from helpers.JsonFileHandler import JsonFileHandler
from LogPublisherNode import LogPublisherNode

@implementer(iSwitching, iLoggable)
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
        self.is_open = False
        self.json_file_handler = JsonFileHandler()
        self.log_publisher = LogPublisherNode()
        
    def open(self) -> None:
        self.gpio.output(self.pin, GPIO.HIGH)
        self.is_open = True
        self.logToFile(LogSeverity.INFO, "Switching open.", "Switching")
        self.logToGUI(LogSeverity.INFO, "Switching open.", "Switching")
        
    def close(self) -> None:
        self.gpio.output(self.pin, GPIO.LOW)
        self.is_open = False
        self.logToFile(LogSeverity.INFO, "Switching close.", "Switching")
        self.logToGUI(LogSeverity.INFO, "Switching close.", "Switching")
        
    def toggle(self) -> None:
        if self.is_open:
            self.close()
            self.logToFile(LogSeverity.INFO, "Switching close.", "Switching")
            self.logToGUI(LogSeverity.INFO, "Switching close.", "Switching")
        else:
            self.open()
            self.logToFile(LogSeverity.INFO, "Switching open.", "Switching")
            self.logToGUI(LogSeverity.INFO, "Switching open.", "Switching")
            
    def is_open(self) -> bool:
        return self.is_open
    
    def is_closed(self) -> bool:
        return not self.is_open
    
    def logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log.toDictionary())
        return log
    
    def logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name)
        return log


   