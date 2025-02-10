#!/usr/bin/env python3

import rospy
import time
from services.Navigation import Navigation
from services.Vectorizer import Vectorizer
from control.msg import Joystick
from services.Joystick import CJoystick


class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=False)
        joystick = CJoystick()

    def run(self):
        rospy.Subscriber("/joystick", Joystick, self.navigate)
        rospy.spin()

    def navigate(self, data: Joystick):
        x_axis = data.left_x_axis
        y_axis = 0
        z_axis = data.left_y_axis
        yaw_axis = data.right_x_axis
        pitch_axis = data.right_y_axis 
        Navigation().navigate(x_axis, y_axis, z_axis, y_axis, pitch_axis)

if __name__ == "__main__":
    try:
        node = NavigationNode()
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")