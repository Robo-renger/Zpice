#!/usr/bin/env python3

import rospy
import time
from services.Navigation import Navigation
from services.Vectorizer import Vectorizer

def test_navigation():

    try:
        rospy.loginfo("Testing navigate function with joystick inputs...")

        Vectorizer.yaw_only = False
        Navigation.navigate(0.5, 0.5, 1, 1, 1)
        
        # Navigation.moveUp(50)
        # Navigation.moveDown(50)
        # Navigation.moveRight(50)
        # Navigation.moveLeft(50)
        # Navigation.moveForward(100)  
        # Navigation.moveBackward(50)
        # Navigation.rotateClockwise(50)4
        # Navigation.rotateAnticlockwise(75)
        
    except KeyboardInterrupt:
        Navigation.stopAll()
        print("Exiting ...")
    

if __name__ == "__main__":
    try:
        rospy.init_node("navigation_test_node")

        rospy.loginfo("Starting Navigation Test Node...")

        test_navigation()
        # rospy.spin()
        rospy.loginfo("Navigation Test ssCompleted.")
        # time.sleep(10)
        # Navigation.stopAll()
    except KeyboardInterrupt:
        Navigation.stopAll()
        print("Exiting ...")
