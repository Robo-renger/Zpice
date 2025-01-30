#!/usr/bin/env python3

import rospy
from services.Navigation import Navigation

def test_navigation():
    rospy.loginfo("Testing navigate function with joystick inputs...")

    Navigation.navigate(1, 1, 1, -1, 0)
    

if __name__ == "__main__":
    rospy.init_node("navigation_test_node")

    rospy.loginfo("Starting Navigation Test Node...")

    test_navigation()

    rospy.loginfo("Navigation Test Completed.")
