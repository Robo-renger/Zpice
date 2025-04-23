#!/usr/bin/env python3
import rospy 
from services.Rain import Rain
from control.msg import WaterDetection
from exceptions.SensorReadError import SensorReadError
from exceptions.SensorInitializationError import SensorInitializationError
from utils.Configurator import Configurator

class RainNode:
    def __init__(self):
        rospy.init_node('rain_node', anonymous=False)
        self.__pins = Configurator().fetchData(Configurator().PINS)
        self.rain_pin = self.__pins['RAIN_GPIO']
        self.rain_sensor = Rain(self.rain_pin)
        self.pub = rospy.Publisher("Rain", WaterDetection, queue_size=10)
        self.msg = WaterDetection()

    def run(self):
        try:
            water_status = self.rain_sensor.checkWater()
            self.msg.status = water_status
            self.pub.publish(self.msg)
        except SensorReadError as e:
            rospy.logerr(f"Error reading the rain sensor: {e}")
    
if __name__ == "__main__":
    try:
        node = RainNode()
        rate = rospy.Rate(70)
        while not rospy.is_shutdown():
            node.run()
            rate.sleep()
    except SensorInitializationError as e:
        rospy.logerr(f"Sensor initialization failed. {e}")
    except KeyboardInterrupt:
        print("Stopping...")
    

    