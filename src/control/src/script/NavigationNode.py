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
        axis = self.joystick.getAxis()
        left_x  = float(axis['left_x_axis'])
        left_y  = float(axis['left_y_axis'])
        right_x = float(axis['right_x_axis'])
        right_y = float(axis['right_y_axis'])

        Navigation().navigate(left_x, left_y, 0, right_y, right_x)

if __name__ == "__main__":
    try:
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")