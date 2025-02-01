#!/usr/bin/env python3

from zope.interface import implementer
from interface.iSwitching import iSwitching

SIMULATION_MODE = False
try:
    import RPi.GPIO as GPIO
    
except (RuntimeError, ImportError):
    SIMULATION_MODE = True
    print("RPi.GPIO not found. Running in simulation mode.")
    

@implementer(iSwitching)
class Switching:
    def __init__(self, pin, gpio_interface=None):
        """
        Initialize the Switching object With specified pin.
        gpio_interface used for mocking the GPIO library.
        """
        self.simulation_mode = SIMULATION_MODE
        # self.gpio = gpio_interface if gpio_interface else (GPIO if not self.simulation_mode else None)
        if not self.simulation_mode:
            self.gpio = GPIO
            self.gpio.setmode(GPIO.BCM)
            self.gpio.setup(pin, GPIO.OUT)
            self.gpio.output(pin, GPIO.LOW) # Initialize the pin to low
        else:
            self.gpio = None
        self.pin = pin
        self._is_open = False
        
    def open(self) -> None:
        if not self.simulation_mode:
            self.gpio.output(self.pin, GPIO.HIGH)
        else:
            print("Simulating opening the switch")
        self.is_open = True
        
    def close(self) -> None:
        if not self.simulation_mode:
            self.gpio.output(self.pin, GPIO.LOW)
        else:
            print("Simulating closing the switch")
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
