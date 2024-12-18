from zope.interface import implementer
from interface.PWMDriver import PWMDriver

@implementer(PWMDriver)
class PCA:
    __inst = None

    def __init__(self, i2c_address=0x40, frequency=50, simulation_mode=False):
        self.simulation_mode = simulation_mode
        self.__initializePCA(i2c_address, frequency)

    def __initializePCA(self, i2c_address, frequency):
        """
        Initialize the PCA9685 driver.
        """
        if self.simulation_mode:
            print("Running in simulation mode. PCA9685 not initialized.")
        else:
            try:
                import board
                import busio
                from adafruit_pca9685 import PCA9685
                self.board_avaliable = True
                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(i2c, address=i2c_address)
                self.pca.frequency = frequency
            except (RuntimeError, ImportError):
                self.simulation_mode = True

    def _microsecondsToDutycycle(self, microseconds):
        """
        Converts microseconds to a duty cycle value.
        """
        period_us = 1_000_000 / self.pca.frequency
        duty_cycle = int((microseconds / period_us) * 65535)
        return duty_cycle

    def PWMWrite(self, channel, microseconds):
        """
        Set the PWM duty cycle for a specific channel based on the pulse width in microseconds.
        """
        if not 0 <= channel <= 15:
            raise ValueError("Channel must be between 0 and 15.")

        duty_cycle_value = self._microsecondsToDutycycle(microseconds)
        self.pca.channels[channel].duty_cycle = duty_cycle_value

    def stopAll(self):
        """
        Stop PWM output on all channels.
        """
        for channel in self.pca.channels:
            channel.duty_cycle = 0

    def close(self):
        self.pca.deinit()

    @staticmethod
    def getInst(simulation_mode=False):
        """
        Get or create the singleton instance.
        """
        if PCA.__inst is None:
            PCA.__inst = PCA(simulation_mode=simulation_mode) # added simulation_mode parameter
        return PCA.__inst
    


if __name__ == "__main__":
    driver = PCA.getInst(simulation_mode=False) # added simulation_mode parameter
    driver.PWMWrite(0, 1500)
    driver.close()
