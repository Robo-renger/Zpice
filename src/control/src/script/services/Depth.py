#!/usr/bin/env python3
from ms5837 import MS5837_02BA
from zope.interface import implementer
from interface.IDepth import IDepthSensor
from interface.iLoggable import iLoggable
from exceptions.SensorInitializationError import SensorInitializationError
from exceptions.SensorReadError import SensorReadError
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from helpers.JsonFileHandler import JsonFileHandler
from script.LogPublisherNode import LogPublisherNode

@implementer(IDepthSensor, iLoggable)
class DepthSensor:
    def __init__(self, bus: int = 0, fluid_density: int = 1000) -> None:
        """
        Initializes the DepthSensor. 
        Args:
            bus (int): The bus number for the sensor.
            fluid_density (int): The density of the fluid (default is 1000 kg/m³ for freshwater).
        Raises:
            SensorInitializationError: If the sensor fails to initialize.
        """
        self.json_file_handler = JsonFileHandler()
        self.log_publisher = LogPublisherNode()
        self.sensor = MS5837_02BA(bus) # Specify bus

        if not self.sensor.init():
            self.logToFile(LogSeverity.ERROR, "Depth sensor initialization failed.", "DepthSensor")
            self.logToGUI(LogSeverity.ERROR, "Depth sensor initialization failed.", "DepthSensor")
            raise SensorInitializationError("Depth sensor initialization failed.")
        
        self._fluid_density = fluid_density

    def setFluidDensity(self, density: int) -> None:
        """Sets the fluid density incase of different environmets like salt water.
        @param density: density of the fluid the sensor will be soaked in."""
        self._fluid_density = density

    def getFluidDensity(self) -> int:
        """Get the already set fluid density
        @return fluid density"""
        return self._fluid_density

    def readData(self) -> None:
        """
        Reads data from the sensor.
        Raises: SensorReadError: If the sensor fails to read data.
        """
        try:
            self.sensor.read()
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to read sensor data: {str(e)}", "DepthSensor")
            self.logToGUI(LogSeverity.ERROR, f"Failed to read sensor data: {str(e)}", "DepthSensor")
            raise SensorReadError(f"Failed to read depth sensor data: {str(e)}")
    
    def getPressure(self) -> float:
        """
        Returns the pressure reading from the sensor.
        Raises: SensorReadError: If the sensor fails to read pressure data.
        """
        try:
            return self.sensor.pressure()
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to read pressure data: {str(e)}", "DepthSensor")
            self.logToGUI(LogSeverity.ERROR, f"Failed to read pressure data: {str(e)}", "DepthSensor")
            raise SensorReadError(f"Failed to read pressure data: {str(e)}")

    def getDepth(self) -> float:
        """
        Returns the depth reading from the sensor.
        Raises: SensorReadError: If the sensor fails to read depth data.
        """
        try: 
            return self.sensor.depth()
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to read depth data: {str(e)}", "DepthSensor")
            self.logToGUI(LogSeverity.ERROR, f"Failed to read depth data: {str(e)}", "DepthSensor")
            raise SensorReadError(f"Failed to read depth data: {str(e)}")
    
    def logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log.toDictionary())
        return log
    
    def logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name)
        return log

    

    