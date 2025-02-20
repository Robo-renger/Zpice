#!/usr/bin/env python3

import rospy
from services.Navigation import Navigation
from services.Joystick import CJoystick
from services.Vectorizer import Vectorizer

import time

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=False)
        self.joystick = CJoystick()
        self.x = 0
        self.y = 0
        self.z = 0
        self.pitch = 0
        self.yaw = 0
        self.last_reset_time = 0  

    def navigate(self):
        # Vectorizer.yaw_only = True
        current_time = time.time()
        axis_values = self.joystick.getAxis()
        self.x = axis_values.get('left_x_axis', 0)
        self.y = axis_values.get('left_y_axis', 0)
        self.pitch = axis_values.get('right_y_axis', 0)
        self.yaw = axis_values.get('right_x_axis', 0)
        if self.joystick.isClicked("HEAVE_DOWN") and self.joystick.isClicked("HEAVE_UP"):
            self.z = 0.0
            self.last_reset_time = current_time 

        elif current_time - self.last_reset_time > 0.2:  
            if self.joystick.isClicked("HEAVE_UP"):
                self.z = min(self.z + 0.01, 1)
            elif self.joystick.isClicked("HEAVE_DOWN"):
                self.z = max(self.z - 0.01, -1)
        # rospy.loginfo(f"X TRAVERSAL = {self.x}")
        # rospy.loginfo(f"Y TRAVERSAL = {self.y}")
        # rospy.loginfo(f"Z HEAVE = {self.z}")
        # rospy.loginfo(f"PITCH = {self.pitch}")
        # rospy.loginfo(f"YAW = {self.yaw}")
        Navigation().navigate(self.x, self.y,self.z, self.pitch, self.yaw)
if __name__ == "__main__":
    try:
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Navigation Node...")