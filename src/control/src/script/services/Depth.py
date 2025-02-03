#!/usr/bin/env python3
from ms5837 import MS5837_02BA
from zope.interface import implementer
from interface.IDepth import IDepthSensor
@implementer(IDepthSensor)
class DepthSensor:
    def __init__(self, bus: int = 0, fluid_density: int = 1000) -> None:
        self.sensor = MS5837_02BA(bus) # Specify bus
        if not self.sensor.init():
            print("Sensor couldn't be initialized.\n Exiting...")
            exit()
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
        """Read the sensor data and update the temperature and pressure readings."""
        self.sensor.read()
    
    def getPressure(self) -> float:
        """Get the pressure.
        @returns pressure reading."""
        return self.sensor.pressure()
    
    def getDepth(self) -> float:
        """Get the depth.
        @returns depth reading."""
        return self.sensor.depth()
    
    



    

    