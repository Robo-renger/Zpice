#!/usr/bin/env python3
import rospy
from services.Depth import DepthSensor
from control.msg import Depth

class DepthNode:
    def __init__(self):
        rospy.init_node("depth_node", anonymous=False)
        self.sensor = DepthSensor(3)
        self.pub = rospy.Publisher('depth', Depth, queue_size=10)
        self.msg = Depth()

    def run(self):
        self.sensor.readData()
        self.msg.pressure = self.sensor.getPressure()
        self.msg.depth = self.sensor.getDepth()
        self.pub.publish()

if __name__ == "__main__":
    try:
        node = DepthNode()
        while not rospy.is_shutdown():
            node.run()
    except KeyboardInterrupt:
        print("Exiting...")

    

    