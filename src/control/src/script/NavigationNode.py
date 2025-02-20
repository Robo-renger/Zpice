#!/usr/bin/env python3

import rospy
from control.msg import IMU, Depth
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
        self.pid_heave = PIDController(0.1, 0.1, 0.1)
        
        self.current_heading = 0.0
        self.current_pitch = 0.0
        self.current_depth = 0.0
        
        self.imu_data = {
            'pitch': 0.0,
            'yaw' : 0.0
        }
        self.depth = 0.0
        
        rospy.Subscriber("IMU", IMU, self._imuCallback)
        rospy.Subscriber("depth", Depth, self._depthCallback)

    def _imuCallback(self, msg: IMU):
        self.imu_data['pitch'] = msg.pitch
        self.imu_data['yaw'] = msg.yaw

    def _depthCallback(self, msg: Depth):
        self.depth = msg.depth

    def navigate(self):
        current_time = time.time()
        axis_values = self.joystick.getAxis()
        self.x = axis_values.get('left_x_axis', 0)
        self.y = -1 * axis_values.get('left_y_axis', 0)
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

        # Update current heading, pitch, and depth when moving
        if self.x != 0 or self.y != 0 or self.z != 0 or self.pitch != 0 or self.yaw != 0:
            self.current_heading = self.imu_data['yaw']
            self.current_pitch = self.imu_data['pitch']
            self.current_depth = self.depth

        # No joystick inputs (ROV at Rest)
        if self.x == 0 and self.y == 0 and self.z == 0 and self.pitch == 0 and self.yaw == 0:
            yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
            pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
            depth_output = self.pid_heave.stabilize(self.depth)
            Navigation.navigate(0, 0, depth_output, pitch_output, yaw_output)
        else:
            # Joystick input is present, stabilize heading, pitch, and depth if no input is provided
            if self.yaw == 0:
                self.pid_yaw.updateSetpoint(self.current_heading)
                yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
            else:
                yaw_output = self.yaw

            if self.pitch == 0:
                self.pid_pitch.updateSetpoint(self.current_pitch)
                pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
            else:
                pitch_output = self.pitch

            if self.z == 0:
                self.pid_heave.updateSetpoint(self.current_depth)
                depth_output = self.pid_heave.stabilize(self.depth)
            else:
                depth_output = self.z

            Navigation.navigate(self.x, self.y, depth_output, pitch_output, yaw_output)
        # rospy.loginfo(f"X TRAVERSAL = {self.x}")
        # rospy.loginfo(f"Y TRAVERSAL = {self.y}")
        # rospy.loginfo(f"Z HEAVE = {self.z}")
        # rospy.loginfo(f"PITCH = {self.pitch}")
        # rospy.loginfo(f"YAW = {self.yaw}")
        # Navigation().moveForward(80)

        Navigation().navigate(self.x, self.y, self.pitch, self.z, self.yaw)

if __name__ == "__main__":
    try:
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Navigation Node...")