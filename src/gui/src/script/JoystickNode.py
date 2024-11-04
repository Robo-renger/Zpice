#!/usr/bin/env python3

import rospy
from gui.msg import Joystick
class Joystick:
    def __init__(self):
        rospy.init_node('joystick_filter', anonymous=True)
        self.data
    def callback(self, data):
        self.data = data
        print(data)
    def publishAxis(self):
        axis_data
    def run(self):  
        rospy.Subscriber("/joystick", Joystick, self.callback)
        rospy.spin()

if __name__ == "__main__":
    node = JoystickNode()
    node.run()
