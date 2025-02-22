#!/usr/bin/env python3

import rospy
import time
from services.Navigation import Navigation
from services.Vectorizer import Vectorizer
from utils.Configurator import Configurator

class TestNavigationNode:
    
    def reload(self):
        print(Configurator().fetchData(Configurator.CAMERAS))

    def test_navigation(self):
        try:
            while not rospy.is_shutdown():
                rospy.loginfo("Testing navigate function with joystick inputs...")

                # Vectorizer.yaw_only = False
                # Navigation.navigate(0.5, 0.5, 1, 1, 1)

                Navigation.moveRight(50)
                # Navigation.moveLeft(50)
                # Navigation.moveUp(50)
                # Navigation.moveDown(50)
                # Navigation.moveForward(100)  
                # Navigation.moveBackward(50)
                # Navigation.rotateClockwise(50)4
                # Navigation.rotateAnticlockwise(75)
                rospy.loginfo("Waiting for restart")
                time.sleep(5)  # Delay to avoid excessive printing
        except KeyboardInterrupt:
            print("Exiting ...")

if __name__ == "__main__":
    rospy.init_node("navigation_test_node")  # Initialize the node once
    TestNavigationNode().test_navigation()  # Keep printing continuously
