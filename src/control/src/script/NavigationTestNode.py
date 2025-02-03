#!/usr/bin/env python3

import rospy
from services.Navigation import Navigation
from services.Vectorizer import Vectorizer

def test_navigation():
    rospy.loginfo("Testing navigate function with joystick inputs...")

    Vectorizer.yaw_only = True
    Navigation.navigate(0, 0, 0, 0, 0.5)
    # Navigation.moveUp(50)
    # Navigation.moveDown(50)
    # Navigation.moveRight(100)
    # Navigation.moveLeft(100)
    # Navigation.moveForward(50)
    # Navigation.moveBackward(50)
    # Navigation.rotateClockwise(75)
    # Navigation.rotateAnticlockwise(75)
    

if __name__ == "__main__":
    rospy.init_node("navigation_test_node")

    rospy.loginfo("Starting Navigation Test Node...")

    test_navigation()

    rospy.loginfo("Navigation Test Completed.")
