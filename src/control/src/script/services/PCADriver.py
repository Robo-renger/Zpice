#!/usr/bin/env python3
from zope.interface import implementer
from interface.PWMDriver import PWMDriver
from utils.EnvParams import EnvParams
import board
import busio
from adafruit_pca9685 import PCA9685
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from script.LogPublisherNode import LogPublisherNode
from helpers.JsonFileHandler import JsonFileHandler

@implementer(PWMDriver)
class PCA:
    __inst = None

    def __init__(self, i2c_address=0x40, frequency=50):
        self.__simulation_mode =  EnvParams().ENV == "SIMULATION"
        self.__initializePCA(i2c_address, frequency)
        self.frequency = frequency
        self.log_publisher = LogPublisherNode()
        self.json_file_handler = JsonFileHandler()

    def __initializePCA(self, i2c_address, frequency):
        """
        Initialize the PCA9685 driver.

        raises:
            RuntimeError: If the PCA9685 driver fails to initialize.
            ImportError: If the required libraries are not installed.
        """
        if self.__simulation_mode:
            print("Running in simulation mode. PCA9685 not initialized.")
            self.pca = type('DummyPCA', (), {
                'frequency': frequency})
        else:
            try:
                self.board_avaliable = True
                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(i2c, address=i2c_address)
                self.pca.frequency = frequency
            except (RuntimeError, ImportError):
                self.__logToFile(LogSeverity.ERROR,"Couldnt find PCA on i2c bus, while Environemnt is not set to 'SIMULATION'")
                self.__logToGUI(LogSeverity.ERROR,"Couldnt find PCA on i2c bus, while Environemnt is not set to 'SIMULATION'")

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

        raises:
            ValueError: If the channel is not between 0 and 15.
        """

        if not 0 <= channel <= 15:
            self.logToFile(LogSeverity.ERROR, "Channel must be between 0 and 15.", "PCA9685")
            self.logToGUI(LogSeverity.ERROR, "Channel must be between 0 and 15.", "PCA9685")
            raise ValueError("Channel must be between 0 and 15.")

        if self.__simulation_mode:
            print(
                f"[Simulation Mode] Setting channel {channel} to {microseconds} microseconds.")
        else:
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
    def getInst():
        """
        Get or create the singleton instance.
        """
        if PCA.__inst is None:
            # added simulation_mode parameter
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