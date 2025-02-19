#!/usr/bin/env python3
import subprocess
import psutil 
import rospy
import time
import pigpio
from zope.interface import implementer
from interface.IPiHealth import IPiHealth
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity

@implementer(IPiHealth)
class PiHealth:
    """Static class to get raspberry status"""
    @staticmethod
    def getTemp() -> float:
        """Gets the temperature of the Pi
        @returns temp of the pi"""
        try:
            temp_output = subprocess.check_output(['vcgencmd', 'measure_temp']).decode('utf-8')
            temp = float(temp_output.replace('temp=', '').replace("'C\n", ''))
            Logger.logToFile(LogSeverity.INFO, f"Temperature = {str(temp)}", "Pi")
            Logger.logToGUI(LogSeverity.INFO, f"Temperature = {str(temp)}", "Pi")
            return temp
        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Error getting temperature: {str(e)}", "Pi")
            Logger.logToGUI(LogSeverity.ERROR, f"Error getting temperature: {str(e)}", "Pi")
            return None

    @staticmethod
    def checkUndervoltage() -> bool:
        """Checks whether the raspberry is going through undervoltage or not
        @returns true incase of undervoltage has been detected"""
        try:
            throttled = subprocess.check_output(['vcgencmd', 'get_throttled']).decode('utf-8')
            undervoltage_flags = (1 << 0) | (1 << 16)
            status = int(throttled.split('=')[1], 16)
            # Define undervoltage conditions:
                # Bit 0 (0x1): Undervoltage currently detected.
                # Bit 16 (0x10000): Undervoltage has occurred since boot.
                # Bit 16 (0x50005): Undervoltage and throttling have been detected
            if (status & undervoltage_flags):
                Logger.logToFile(LogSeverity.WARNING, f"Undervoltage detected: {str(throttled)}", "Pi")
                Logger.logToGUI(LogSeverity.WARNING, f"Undervoltage detected: {str(throttled)}", "Pi")
                return True
            else:
                Logger.logToFile(LogSeverity.INFO, f"No undervoltage detected: {str(throttled)}", "Pi")
                Logger.logToGUI(LogSeverity.INFO, f"No undervoltage detected: {str(throttled)}", "Pi")
                return False

        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Error checking undervoltage: {str(e)}", "Pi")
            Logger.logToGUI(LogSeverity.ERROR, f"Error checking undervoltage: {str(e)}", "Pi")  
            return None
        
    @staticmethod
    def getRamUsage() -> tuple:
        """Monitors the usage space of the ram
        @returns total memory of the raspberry
        @returns used memory
        @returns free space of the memory
        @returns percentage of free space"""
        try:
            mem = psutil.virtual_memory()
            total = mem.total / (1024 ** 3)
            used = (mem.total - mem.available) / (1024 ** 3)
            free = float(f"{mem.available / (1024 ** 3):.2f}")
            percent_free =  float(f"{(free / total) * 100:.2f}")
            Logger.logToFile(LogSeverity.INFO, f"total = {total:.2f} GB, used = {used:.2f} GB, free = {free} GB, percent_free = {percent_free}%", "Pi")
            Logger.logToGUI(LogSeverity.INFO, f"total = {total:.2f} GB, used = {used:.2f} GB, free = {free} GB, percent_free = {percent_free}%", "Pi")
            return (free, percent_free)
        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Error in getting ram usage: {str(e)}", "Pi")
            Logger.logToGUI(LogSeverity.ERROR, f"Error in getting ram usage: {str(e)}", "Pi")  
            return (None, None) 
    
    @staticmethod
    def getCPUUsage() -> float:
        """Monitors the usage of the CPU
        @returns percentage of used CPU"""
        try:
            output = subprocess.check_output("top -bn1 | grep 'Cpu(s)' | awk '{print 100 - $8}'", shell=True).decode('utf-8')
            Logger.logToFile(LogSeverity.INFO, f"Total CPU Usage: {output.strip()}", "Pi")
            Logger.logToGUI(LogSeverity.INFO, f"Total CPU Usage: {output.strip()}", "Pi")
            return float(output.strip())
        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Error in getting CPU usage: {str(e)}", "Pi")
            Logger.logToGUI(LogSeverity.ERROR, f"Error in getting CPU usage: {str(e)}", "Pi")  
            return None

    @staticmethod
    def checkCoreVoltage() -> bool:
        """Checks if the voltages across the GPU core and the SDRAM rails are within normal ranges.
        Typical nominal values on a raspberry pi4: around 1.3 V (voltage >= 1.2V is normal) 
        @return core_state: True if core voltage is within normal range."""
        try:
            core_output = subprocess.check_output(['vcgencmd', 'measure_volts', 'core']).decode('utf-8').strip()
            core_voltage = float(core_output.replace("volt=", "").replace("V", ""))
            core_state = core_voltage >= 1.2
            Logger.logToFile(LogSeverity.INFO, f"GPU core voltage = {str(core_voltage)}", "Pi")
            Logger.logToGUI(LogSeverity.INFO, f"GPU core voltage = {str(core_voltage)}", "Pi")
            return core_state
        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Error in getting checking GPU core voltage: {str(e)}", "Pi")
            Logger.logToGUI(LogSeverity.ERROR, f"Error in getting checking GPU core voltage: {str(e)}", "Pi")  
            return None
        
    @staticmethod
    def checkSDRAMVoltage() -> bool:
        """Checks if the voltages across the SDRAM Rails are within normal ranges.
        Trypical nominal values on a raspberry pi4:
            - sdram_c, sdram_i, sdram_p: around 1.225-1.25 V (voltages >= 1.1V are normal).
        @return sdram_state: True if all SDRAM rails are within normal range."""
        try:
            sdram_state = True
            for rail in ["sdram_c", "sdram_i", "sdram_p"]:
                rail_output = subprocess.check_output(['vcgencmd', 'measure_volts', rail]).decode('utf-8').strip()
                voltage = float(rail_output.replace("volt=", "").replace("V", ""))
                if voltage < 1.1:
                    sdram_state = False
                Logger.logToFile(LogSeverity.INFO, f"Voltage on {rail} = {str(voltage)}", "Pi")
                Logger.logToGUI(LogSeverity.INFO, f"Voltage on {rail} = {str(voltage)}", "Pi")
            return sdram_state
        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Error in checking SDRAM voltage: {str(e)}", "Pi")
            Logger.logToGUI(LogSeverity.ERROR, f"Error in checking SDRAM voltage: {str(e)}", "Pi")  
            return None

    @staticmethod
    def getGPIOStatus(pi: pigpio.pi) -> dict:
        """Gets the realtime status of all the 27GPIOs eihter input or output and incase of output does it have a high or low.
        @return dictionary where each key is a GPIO number (int) and the value is another dict: { "mode": str, "level": str }"""
        try:
            gpio_status = {}
            if not pi.connected:
                Logger.logToFile(LogSeverity.ERROR, f"Could not connect to pigpio daemon. Please start it with 'sudo pigpiod'", "Pi")
                Logger.logToGUI(LogSeverity.ERROR, f"Could not connect to pigpio daemon. Please start it with 'sudo pigpiod'", "Pi")
                return None
            for pin in range(0, 28):
                mode_val = pi.get_mode(pin)
                level_val = pi.read(pin)
 
                if mode_val == 0:
                    mode_str = "INPUT"
                elif mode_val == 1:
                    mode_str = "OUTPUT"
                else:
                    mode_str = f"ALT{mode_val - 2}"
                    
                level_str = "HIGH" if level_val == 1 else "LOW"
                
                gpio_status[pin] = {"mode": mode_str, "level": level_str}

            Logger.logToFile(LogSeverity.INFO, f"GPIO Status: {gpio_status}", "Pi")
            Logger.logToGUI(LogSeverity.INFO, f"GPIO Status: {gpio_status}", "Pi")
            return gpio_status
        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Error in getting GPIO status: {str(e)}", "Pi")
            Logger.logToGUI(LogSeverity.ERROR, f"Error in getting GPIO status: {str(e)}", "Pi")  
            return None
