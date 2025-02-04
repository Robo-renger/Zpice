#!/usr/bin/env python3
from zope.interface import Interface

class IDepthSensor(Interface):
    def __init__(self, bus: int, fluid_density: int) -> None:
        """Initialize the sensor.
        @param bus: the bus number the sensor is connected to.
        @param fluid_density: the density of the fluid the sensor will be put in."""

    def setFluidDensity(self, density: int) -> None:
        """Sets the fluid density incase of different environmets like salt water.
        @param density: density of the fluid the sensor will be soaked in."""

    def getFluidDensity(self) -> int:
        """Get the already set fluid density
        @return fluid density"""

    def readData(self) -> None:
        """Read the sensor data and update the temperature and pressure readings."""

    def getPressure(self) -> float:
        """Get the pressure.
        @returns pressure reading."""

    def getDepth(self) -> float:
        """Get the depth.
        @returns depth reading."""
