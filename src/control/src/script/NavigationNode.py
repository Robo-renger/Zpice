#!/usr/bin/env python3

import rospy
from control.msg import IMU
from services.Joystick import CJoystick
from services.Navigation import Navigation
from services.Vectorizer import Vectorizer
from services.PIDController import PIDController
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
        
        self.pid_yaw = PIDController(0.1, 0.1, 0.1)
        self.pid_pitch = PIDController(0.1, 0.1, 0.1)
        self.imu_data = {
            'pitch': 0.0,
            'yaw' : 0.0
        }
        rospy.Subscriber("IMU", IMU, self._imuCallback)

    def _imuCallback(self, msg: IMU):
        self.imu_data['pitch'] = msg.pitch
        self.imu_data['yaw'] = msg.yaw

    def navigate(self):
        # Vectorizer.yaw_only = False
        current_time = time.time()
        axis_values = self.joystick.getAxis()
        self.x = axis_values.get('left_x_axis', 0)
        self.y = axis_values.get('left_y_axis', 0)
        self.pitch = axis_values.get('right_y_axis', 0)
        self.yaw = axis_values.get('right_x_axis', 0)
        if self.joystick.isPressed("HEAVE_DOWN") and self.joystick.isPressed("HEAVE_UP"):
            self.z = 0.0
            self.last_reset_time = current_time 
        elif current_time - self.last_reset_time > 0.2:  
            if self.joystick.isPressed("HEAVE_UP"):
                self.z = min(self.z + 0.07, 1)
            elif self.joystick.isPressed("HEAVE_DOWN"):
                self.z = max(self.z - 0.07, -1)
        # rospy.loginfo(f"X TRAVERSAL = {self.x}")
        # rospy.loginfo(f"Y TRAVERSAL = {self.y}")
        # rospy.loginfo(f"Z HEAVE = {self.z}")
        # rospy.loginfo(f"PITCH = {self.pitch}")
        # rospy.loginfo(f"YAW = {self.yaw}")
        # Navigation().moveForward(80)

        if self.x == 0 and self.y == 0 and self.z == 0 and self.pitch == 0 and self.yaw == 0:
            yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
            pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
            Navigation.navigate(0, 0, 0, pitch_output, yaw_output)
        else:
            self.pid_yaw.updateSetpoint(self.imu_data['yaw'])
            self.pid_pitch.updateSetpoint(self.imu_data['pitch'])
            Navigation.navigate(self.x, self.y, self.z, self.pitch, self.yaw)

if __name__ == "__main__":
    try:
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Navigation Node...")