#!/usr/bin/env python3
from zope.interface import Interface

class IPiHealth(Interface):
    """Static class to get raspberry status"""
    
    def getTemp() -> float:
        """Gets the temperature of the Pi
        @returns temp of the pi"""
        pass

    
    def checkUndervoltage() -> bool:
        """Checks whether the raspberry is going through undervoltage or not
        @returns true incase of undervoltage has been detected"""
        pass

    
    def getRamUsage() -> tuple:
        """Monitors the usage space of the ram
        @returns used memory
        @returns free space of the memory
        @returns percentage of free space"""
        pass

    
    def getCPUUsage() -> float:
        """Monitors the usage of the CPU
        @returns percentage of used CPU"""
        pass

    
    def checkCoreVoltage() -> bool:
        """Checks if the voltages across the GPU core and the SDRAM rails are within normal ranges.
        Typical nominal values on a raspberry pi4: around 1.3 V (voltage >= 1.2V is normal) 
        @return core_state: True if core voltage is within normal range."""
        pass

    
    def checkSDRAMVoltage() -> bool:
        """Checks if the voltages across the SDRAM Rails are within normal ranges.
        Trypical nominal values on a raspberry pi4:
            - sdram_c, sdram_i, sdram_p: around 1.225-1.25 V (voltages >= 1.1V are normal).
        @return sdram_state: True if all SDRAM rails are within normal range."""
        pass

    
    def getGPIOStatus() -> dict:
        """Gets the realtime status of all the 27GPIOs eihter input or output and incase of output does it have a high or low.
        @return dictionary where each key is a GPIO number (int) and the value is another dict: { "mode": str, "level": str }"""
        pass
        

    
    
