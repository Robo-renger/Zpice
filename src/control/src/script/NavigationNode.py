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

    def handleJoystickInput(self):
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
            self.pid_yaw.updateSetpoint(self.imu_data['yaw'])
            self.pid_pitch.updateSetpoint(self.imu_data['pitch'])
            self.pid_heave.updateSetpoint(self.depth)

    def stabilizeAtRest(self):
        """
        Stabilize yaw and pitch when the ROV is at rest.
        """
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        Navigation.navigate(0, 0, pitch_output, 0, yaw_output)

    def fixHeave(self):
        """
        Stabilize depth (heave) when the ROV is moving horizontally.
        """
        depth_output = self.pid_heave.stabilize(self.depth)
        Navigation.navigate(self.x, self.y, 0, depth_output, self.yaw)

    def fixHeading(self):
        """
        Stabilize yaw (heading) when the ROV is moving vertically.
        """
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        Navigation.navigate(self.x, self.y, self.pitch, self.z, yaw_output)

    def navigate(self):
        # Vectorizer.yaw_only = False
        self.handleJoystickInput()

        # ROV is at rest
        if self.x == 0 and self.y == 0 and self.z == 0 and self.pitch == 0 and self.yaw == 0:
            self.stabilizeAtRest()
        
        # ROV is moving horizontally (forward, backward, rightward, leftward)
        elif self.z == 0 and self.pitch == 0:
            self.fixHeave()

        # ROV is moving vertically (upward, downward)
        elif self.x == 0 and self.y == 0 and self.yaw == 0:
            self.fixHeading()
        
        # Default: Use joystick input directly
        else:
            Navigation.navigate(self.x, self.y, self.pitch, self.z, self.yaw)

if __name__ == "__main__":
    try:
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Navigation Node...")
