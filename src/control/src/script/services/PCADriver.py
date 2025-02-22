#!/usr/bin/env python3
from zope.interface import implementer
from interface.PWMDriver import PWMDriver
from utils.EnvParams import EnvParams
import board
import busio
from adafruit_pca9685 import PCA9685
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity

@implementer(PWMDriver)
class PCA:
    __inst = None

    def __init__(self, i2c_address=0x40, frequency=50):
        self.__simulation_mode = EnvParams().ENV == "SIMULATION"
        self.frequency = frequency
        self.__initializePCA(i2c_address, self.frequency)

    def __initializePCA(self, i2c_address, frequency):
        """
        Initialize the PCA9685 driver.

        raises:
            RuntimeError: If the PCA9685 driver fails to initialize.
            ImportError: If the required libraries are not installed.
        """
        if self.__simulation_mode:
            print("Running in simulation mode. PCA9685 not initialized.")
            self.pca = self.__createDummyPCA(frequency)
        else:
            try:
                import board
                import busio
                from adafruit_pca9685 import PCA9685
                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(i2c, address=i2c_address)
                self.pca.frequency = frequency
            except (RuntimeError, ImportError):
                Logger.logToFile(LogSeverity.ERROR,"Couldnt find PCA on i2c bus, while Environemnt is not set to 'SIMULATION'")
                Logger.logToGUI(LogSeverity.ERROR,"Couldnt find PCA on i2c bus, while Environemnt is not set to 'SIMULATION'")

    def _microsecondsToDutycycle(self, microseconds):
        """
        Converts microseconds to a duty cycle value.
        """
        period_us = 1_000_000 / self.frequency
        duty_cycle = int((microseconds / period_us) * 65535)
        return duty_cycle
    def PWMWrite(self, channel, microseconds):
        """
        Set the PWM duty cycle for a specific channel based on the pulse width in microseconds.

        raises:
            ValueError: If the channel is not between 0 and 15.
        """
        if not 0 <= channel <= 15:
            Logger.logToFile(LogSeverity.ERROR, "Channel must be between 0 and 15.", "PCA9685")
            Logger.logToGUI(LogSeverity.ERROR, "Channel must be between 0 and 15.", "PCA9685")
            raise ValueError("Channel must be between 0 and 15.")

        elif self.pca is not None:
            duty_cycle_value = self._microsecondsToDutycycle(microseconds)
            self.pca.channels[channel].duty_cycle = duty_cycle_value
        else:
            self.__logToFile(LogSeverity.ERROR, f"Attempted to write to channel {channel} but PCA is not initialized", "PCA")
            self.__logToGUI(LogSeverity.ERROR, f"Attempted to write to channel {channel} but PCA is not initialized", "PCA")

    def stopAll(self):
        """
        Stop PWM output on all channels.
        """
        if self.pca is not None:
            for channel in self.pca.channels:
                channel.duty_cycle = 0
        else:
            print("[Simulation Mode] Stopping all channels (PCA not initialized).")

    def close(self):
        """
        Deinitialize PCA if available.
        """
        if self.pca is not None and not self.__simulation_mode:
            self.pca.deinit()

    @staticmethod
    def getInst():
        """
        Get or create the singleton instance.
        """
        if PCA.__inst is None:
            PCA.__inst = PCA()
        return PCA.__inst