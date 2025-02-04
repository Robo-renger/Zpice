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
        except:
            rospy.logerr(f"Error in getting CPU usage")
            return None
