#!/usr/bin/env python3

import rospy
from services.Navigation import Navigation
from services.Joystick import CJoystick
from services.Vectorizer import Vectorizer


class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=False)
        self.joystick = CJoystick()

    def navigate(self):
        # Vectorizer.yaw_only = True
        x_left, y_left, x_right, y_right = self.joystick.getAxis()
        rospy.loginfo(f"x_left = {x_left}")
        rospy.loginfo(f"y_left = {y_left}")
        rospy.loginfo(f"x_right = {x_right}")
        rospy.loginfo(f"y_right = {y_right}")

        # Navigation().navigate(x_left, 0, y_left, y_right, x_right)

if __name__ == "__main__":
    try:
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Navigation Node...")