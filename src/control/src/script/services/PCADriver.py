#!/usr/bin/env python3
from zope.interface import implementer
from interface.PWMDriver import PWMDriver
from utils.EnvParams import EnvParams
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from script.LogPublisherNode import LogPublisherNode
from helpers.JsonFileHandler import JsonFileHandler

@implementer(PWMDriver, iLoggable)
class PCA:
    __inst = None

    def __init__(self, i2c_address=0x40, frequency=50):
        self.__simulation_mode = EnvParams().ENV == "SIMULATION"
        self.frequency = frequency
        self.log_publisher = LogPublisherNode()
        self.json_file_handler = JsonFileHandler()
        self.pca = None
        self.__initializePCA(i2c_address, frequency)

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
            except (RuntimeError, ImportError) as e:
                self.__logToFile(LogSeverity.ERROR, f"Could not find PCA on I2C bus: {e}", "PCA")
                self.__logToGUI(LogSeverity.ERROR, f"Could not find PCA on I2C bus: {e}", "PCA")
                self.pca = None
##################################### START DUMMY PCA #######################################
    def __createDummyPCA(self, frequency):
        """
        Creates a dummy PCA object for simulation.
        """
        class DummyPCA:
            def __init__(self, frequency):
                self.frequency = frequency
                self.channels = [DummyChannel() for _ in range(16)]
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
                """
                if not 0 <= channel <= 15:
                    raise ValueError("Channel must be between 0 and 15.")
                if channel == 5:
                    print(f"[Simulation Mode] Setting channel {channel} to {microseconds} microseconds pulse width.")

            def stopAll(self):
                """
                Stop PWM output on all channels.
                """
                print("[Simulation Mode] Stopping all channels (PCA not initialized).")
        class DummyChannel:
            def __init__(self):
                    self.duty_cycle = 0
        return DummyPCA(frequency)

##################################### END DUMMY PCA #######################################\
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
            self.logToFile(LogSeverity.ERROR, "Channel must be between 0 and 15.", "PCA9685")
            self.logToGUI(LogSeverity.ERROR, "Channel must be between 0 and 15.", "PCA9685")
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

    def __logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log)
        return log

    def __logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name)
        return log
