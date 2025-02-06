from zope.interface import implementer
from interface.iDCMotor import iDCMotor
import RPi.GPIO as GPIO
import time

@implementer(iDCMotor)
class SinglePWMDCMotor:
    def __init__(self, pca, channel, dir_pin, min_value=0, max_value=255):
        self.pca = pca
        self.channel = channel
        self.dir_pin = dir_pin
        self.min_value = min_value
        self.max_value = max_value
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)  

        self.stop()

    def drive(self, speed: int) -> None:
        """
        Set speed based on the current direction.
        """
        if speed < self.min_value or speed > self.max_value:
            raise ValueError(f"Value must be between {self.min_value} and {self.max_value}.")
        
        speed = self._ensure_bounds(speed) 
        pwm_value = self._convert_8bits_to_12bits(speed)
        self.pca.PWMWrite(self.channel, pwm_value)
        

    def direction(self, direction: str) -> None:
        """
        Set Direction for DC Motor
        Parameters: direction --> 'f' for forward, 'r' for reverse
        """
        if direction not in ["f", "r"]:
            raise ValueError("Invalid direction. Use 'f' for forward or 'r' for reverse.")

        if direction == "f":
            GPIO.output(self.dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)


    def stop(self) -> None:
        """
        Stop the motor.
        """
        self.pca.PWMWrite(self.channel, 0)

    def _ensure_bounds(self, value: int) -> int:
        if value < self.min_value:
            return self.min_value
        elif value > self.max_value:
            return self.max_value
        return value
    
    def _convert_8bits_to_12bits(self, value: int) -> int:
        """
        Convert an 8-bit (0-255) value to a 12-bit (0-4095) value.
        """
        return int((value / 255) * 4095)