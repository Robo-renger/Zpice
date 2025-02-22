#!/usr/bin/env python3

import rospy
import time
from utils.Configurator import Configurator

class TestNavigationNode:
    
    def reload(self):
        print(Configurator().fetchData(Configurator.CAMERAS))

    def test_navigation(self):
        try:
            while not rospy.is_shutdown():
                rospy.loginfo("Waiting for restart")
                time.sleep(5)  # Delay to avoid excessive printing
        except KeyboardInterrupt:
            print("Exiting ...")

if __name__ == "__main__":
    rospy.init_node("navigation_test_node")  # Initialize the node once
    TestNavigationNode().test_navigation()  # Keep printing continuously
