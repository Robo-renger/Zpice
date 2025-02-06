#!/usr/bin/env python3
import subprocess
import psutil 
import rospy
from zope.interface import implementer
from interface.IPiHealth import IPiHealth

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
            rospy.loginfo(f"Temperature = {temp}")
            return temp
        except Exception as e:
            rospy.logerr(f"Error getting temperature: {e}")
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
                rospy.loginfo(f"Undervoltage detected: {throttled}")
                return True
            else:
                rospy.loginfo(f"No undervoltage detected: {throttled}")
                return False

        except Exception as e:  
            rospy.logerr(f"Error checking undervoltage: {e}")
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
            rospy.loginfo(f"total = {total:.2f} GB, used = {used:.2f} GB, free = {free} GB, percent_free = {percent_free}%")
            return (free, percent_free)
        except Exception as e:
            rospy.logerr(f"Error in getting ram usage: {e}")
            return (None, None) 
    
    @staticmethod
    def getCPUUsage() -> float:
        """Monitors the usage of the CPU
        @returns percentage of used CPU"""
        try:
            output = subprocess.check_output("top -bn1 | grep 'Cpu(s)' | awk '{print 100 - $8}'", shell=True).decode('utf-8')
            rospy.loginfo(f"Total CPU Usage: {float(output.strip())}")
            return float(output.strip())
        except Exception as e:
            rospy.logerr(f"Error in getting CPU usage: {e}")
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
            rospy.loginfo(f"GPU core voltage = {core_voltage}")
            return core_state
        except Exception as e:
            rospy.logerr(f"Error in getting checking GPU core voltage: {e}")
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
                rospy.loginfo(f"Voltage on {rail} = {voltage}")
            return sdram_state
        except Exception as e:
            rospy.logerr(f"Error in checking SDRAM voltage: {e}")
            return None
