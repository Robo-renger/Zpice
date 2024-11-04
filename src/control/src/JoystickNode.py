#!/usr/bin/env python3

import rospy
from gui.msg import Joystick
from services.Joystick import CJoystick
class JoystickNode:
    def __init__(self):
        rospy.init_node('joystick_filter', anonymous=True)
        self.joystick = CJoystick()
    def callback(self, data):
        self.joystick.data = data
        # print(data)
    def publishAxis(self):
        
    def run(self):  
        rospy.Subscriber("/joystick", Joystick, self.callback)
        rospy.spin()

if __name__ == "__main__":
    node = JoystickNode()
    node.run()
