from zope.interface import implementer
from interface.iDCMotor import iDCMotor


@implementer(iDCMotor)
class DCMotor:
    def __init__(self, pca, channel1, channel2, min_value=0, max_value=255):
        self.pca = pca
        self.channel1 = channel1
        self.channel2 = channel2
        self.min_value = min_value
        self.max_value = max_value
        self.current_direction = "f"  # Default forward
        self.stop()

    def driveForward(self) -> None:
        self.drive(4095)
        self.direction('f')

    def driveForward(self) -> None:
        self.drive(4095)
        self.direction('r')

    def drive(self, speed: int) -> None:
        """
        Set speed based on the current direction.
        """
        if speed < self.min_value or speed > self.max_value:
            raise ValueError(f"Value must be between {self.min_value} and {self.max_value}.")
        
        speed = self._ensure_bounds(speed) 
        pwm_value = self._convert_8bits_to_12bits(speed)

        if self.current_direction == "f":
            self.pca.PWMWrite(self.channel1, pwm_value)
            self.pca.PWMWrite(self.channel2, 0)
        else:
            self.pca.PWMWrite(self.channel1, 0)
            self.pca.PWMWrite(self.channel2, pwm_value)


    def direction(self, direction: str) -> None:
        """
        Adjust which PWM pin is active.
        """
        if direction not in ["f", "r"]:
            raise ValueError("Invalid direction. Use 'f' for forward or 'r' for reverse.")

        self.current_direction = direction

    def stop(self) -> None:
        """
        Stop the motor by setting both PWM channels to 0.
        """
        self.pca.PWMWrite(self.channel1, 0)
        self.pca.PWMWrite(self.channel2, 0)

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