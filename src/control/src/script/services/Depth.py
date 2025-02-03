#!/usr/bin/env python3
from ms5837 import MS5837_02BA, UNITS_atm
from zope.interface import implementer
class Depth:
    def __init__(self, bus: int = 0, fluid_density: int = 1000):
        self.sensor = MS5837_02BA(bus) # Specify model and bus
        if not self.sensor.init():
            print("Sensor couldn't be initialized.\n Exiting...")
            exit()
        self._fluid_density = fluid_density

    def setFluidDensity(self, density: int) -> None:
        """Sets the fluid density incase of different environmets like salt water.
        @param density: density of the fluid the sensor will be soaked in."""
        self._fluid_density = density

    def readData(self) -> None:
        self.sensor.read()
    
    def getPressure(self) -> float:
        """Get the pressure.
        @returns pressure reading."""
        return self.sensor.pressure(UNITS_atm)
    
    def getDepth(self) -> float:
        """Get the depth.
        @returns depth reading."""
        return self.sensor.depth()
    

if __name__ == "__main__":
    try:
        depth_sensor = Depth(bus=3)
        while True:
            depth_sensor.readData()
            print(f"Depth = {depth_sensor.getDepth()}")
    except KeyboardInterrupt:
        print("Exiting...")
    



    

    