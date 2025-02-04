#!/usr/bin/env python3
from zope.interface import Interface

class IPiHealth(Interface):
    """Static class to get raspberry status"""

    def getTemp() -> float:
        """Gets the temperature of the Pi
        @returns temp of the pi"""

    def checkUndervoltage() -> bool:
        """Checks whether the raspberry is going through undervoltage or not
        @returns true incase of undervoltage has been detected"""

    def RamUsage() -> tuple:
        """Monitors the usage space of the ram
        @returns used memory
        @returns free space of the memory
        @returns percentage of free space"""

    def CPUUsage() -> float:
        """Monitors the usage of the CPU
        @returns percentage of used CPU"""
