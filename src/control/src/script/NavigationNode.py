#!/usr/bin/env python3

import rospy
from services.Navigation import Navigation
from services.Joystick import CJoystick


class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=False)
        self.joystick = CJoystick()

    def navigate(self):
        left_x, left_y, right_x, right_y = self.joystick.getAxis()
        Navigation().navigate(left_x, 0, left_y, right_y, right_x)

if __name__ == "__main__":
    try:
        node = NavigationNode()
        node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")