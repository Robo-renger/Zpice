#!/usr/bin/env python3
import rospy
from services.Pressure import Pressure
from std_msgs.msg import Float32
from exceptions.SensorReadError import SensorReadError

class PressureNode:
    def __init__(self):
        rospy.init_node("pressure_node", anonymous=False)
        self.pressure = Pressure()
        self.pub = rospy.Publisher('pressure', Float32, queue_size=10)
        self.temp_pub = rospy.Publisher('temperature', Float32, queue_size=10)
        self.msg = Float32()
        self.temp_msg = Float32()

    def run(self):
        try:
            self.msg = self.pressure.getPressure()
            self.temp_msg.data = self.pressure.getTemperature()

            self.pub.publish(self.msg)
            self.temp_pub.publish(self.temp_msg)
        
        except SensorReadError as e:
            rospy.logerr(f"{e} Skipping this cycle...")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred: {e}")
            
if __name__ == "__main__":
    try:
        node = PressureNode()
        while not rospy.is_shutdown():
            node.run()
    except KeyboardInterrupt:
        print("Exiting...")