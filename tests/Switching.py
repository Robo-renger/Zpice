from zope.interface import implementer
from iSwitching import iSwitching

try:
    import RPi.GPIO as GPIO
    SIMULATION_MODE = False
except ImportError:
    SIMULATION_MODE = True

@implementer(iSwitching)
class Switching:
    def __init__(self, pin, gpio_interface=None):
        """
        Initialize the Switching object With specified pin.
        gpio_interface used for mocking the GPIO library.
        """
        self.simulation_mode = SIMULATION_MODE
        self.gpio = gpio_interface if gpio_interface else (GPIO if not self.simulation_mode else None)
        if not self.simulation_mode:
            self.gpio.setmode(GPIO.BCM)
            self.gpio.setup(pin, GPIO.OUT)
            self.gpio.output(pin, GPIO.LOW) # Initialize the pin to low
        self.pin = pin
        self.is_open = False
        
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
