from zope.interface import implementer
from interface.PWMDriver import PWMDriver

class PCAChannelMock:
    def __init__(self):
        self.duty_cycle = 0

@implementer(PWMDriver)
class PCAMock:
    """Mock class for the PCA9685 PWM driver utilizing its functionality without the need for any hardware."""
    __inst = None   

    def __init__(self, frequency: int = 50):
        self.frequency = frequency
        self.PCA_channels = [PCAChannelMock() for _ in range(16)]

    def microsecondsToDutycycle(self, microseconds: int) -> int:
        """
        Converts microseconds to a duty cycle value.
        """
        period_us = 1_000_000 / self.frequency
        duty_cycle = int((microseconds / period_us) * 65535)
        return duty_cycle

    def PWMWrite(self, channel: int, microseconds: int) :
        """
        Set the PWM duty cycle for a specific channel based on the pulse width in microseconds.
        """
        if not 0 <= channel <= 15:
            raise ValueError("Channel must be between 0 and 15.")

        duty_cycle_value = self.microsecondsToDutycycle(microseconds)
        self.PCA_channels[channel].duty_cycle = duty_cycle_value

    def stopAll(self):
        """
        Stop PWM output on all channels.
        """
        for channel in self.PCA_channels:
            channel.duty_cycle = 0
    
    @staticmethod
    def getInst():
        """
        Get or create the singleton instance.
        """
        if PCAMock.__inst is None:
            PCAMock.__inst = PCAMock()
        return PCAMock.__inst