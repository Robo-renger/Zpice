from zope.interface import implementer
from interface.iSwitching import iSwitching
import RPi.GPIO as GPIO

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
        self.is_open = False
        
    def open(self) -> None:
        self.gpio.output(self.pin, GPIO.HIGH)
        self.is_open = True
        
    def close(self) -> None:
        self.gpio.output(self.pin, GPIO.LOW)
        self.is_open = False
        
    def toggle(self) -> None:
        if self.is_open:
            self.close()
        else:
            self.open()
            
    def is_open(self) -> bool:
        return self.is_open
    
    def is_closed(self) -> bool:
        return not self.is_open
   