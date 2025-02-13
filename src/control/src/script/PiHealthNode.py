#!/usr/bin/env python3
import rospy
from control.msg import Status
from services.PiHealth import PiHealth

class PiHealthNode:
    def __init__(self):
        rospy.init_node('pi_health', anonymous=False)
        self._pub = rospy.Publisher('pihealth', Status, queue_size=10)
        self._msg = Status()
        self._rate = rospy.Rate(1)

    def run(self):
        try:
            temperature = PiHealth.getTemp()
            under_voltage_status = PiHealth.checkUndervoltage()
            free_mem, percent_free_mem = PiHealth.getRamUsage()
            cpu_percentage = PiHealth.getCPUUsage()
            gpu_voltage_state = PiHealth.checkCoreVoltage()
            sdram_voltage_state = PiHealth.checkSDRAMVoltage()
            gpio_status = PiHealth.getGPIOStatus()
            if temperature is not None and under_voltage_status is not None and cpu_percentage is not None and free_mem is not None and gpu_voltage_state is not None and sdram_voltage_state is not None and gpio_status is not None:
                self._msg.temperature = temperature
                self._msg.underVoltage = under_voltage_status
                self._msg.freeMemory = free_mem
                self._msg.percentFreeMemory = percent_free_mem
                self._msg.percentCPUUsage = cpu_percentage
                self._msg.GPUCoreState = gpu_voltage_state
                self._msg.SDRAMState = sdram_voltage_state
                self._msg.GPIOStatus = gpio_status

                self._pub.publish(self._msg)
            self._rate.sleep()
        except Exception as e:
            rospy.logerr(f"Error Publishing pi status: {e}")  

if __name__ == "__main__":
    try:
        node = PiHealthNode()
        while not rospy.is_shutdown():
            node.run()
    except KeyboardInterrupt:
        rospy.logerr("Exiting...")

    