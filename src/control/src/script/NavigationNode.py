#!/usr/bin/env python3

import rospy
from control.msg import IMU, Depth
from std_msgs.msg import String
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
        
        self.pid_yaw = PIDController(0.01, 0, 0)
        self.pid_pitch = PIDController(0.01, 0, 0)
        self.pid_heave = PIDController(0.01, 0, 0)
        
        rospy.Subscriber("IMU", IMU, self._imuCallback)
        rospy.Subscriber("depth", Depth, self._depthCallback)
        
        self.imu_data = {
            'pitch': None,
            'yaw' : None
        }
        self.depth = None

        self.fix_heading = False
        self.fix_heave = False

        rospy.Subscriber("/set", String, self._setFixationCallback)

    def _imuCallback(self, msg: IMU):
        self.imu_data['pitch'] = msg.pitch
        self.imu_data['yaw'] = msg.yaw

    def _depthCallback(self, msg: Depth):
        self.depth = msg.depth

    def _setFixationCallback(self, msg: String):
        if msg.data == "Heading":
            self.fix_heading = True
            self.fix_heave = False
            self.pid_yaw.updateSetpoint(self.imu_data['yaw'])
        elif msg.data == "Heave":
            self.fix_heave = True
            self.fix_heading = False
            self.pid_heave.updateSetpoint(self.depth)
        else:
            self.fix_heading = False
            self.fix_heave = False

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
            if not self.is_vertical and (self.z != 0 or self.pitch != 0):
                self.pid_yaw.updateSetpoint(self.imu_data['yaw'])
                self.is_vertical = True
            self.pid_pitch.updateSetpoint(self.imu_data['pitch'])
            if not self.is_horizontal:
                self.pid_heave.updateSetpoint(self.depth)    
                self.is_horizontal = True

        else:
            self.is_horizontal = False
            self.is_vertical = False

    def stabilizeAtRest(self):
        """
        Stabilize yaw and pitch when the ROV is at rest.
        """
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        Navigation.navigate(0, 0, -pitch_output, 0, -yaw_output)

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
        Navigation.navigate(0, 0, self.pitch, self.z, -yaw_output)

    def navigate(self):
        # Vectorizer.yaw_only = False
        self.handleJoystickInput()

        # ROV is at rest Manual
        if self.x == 0 and self.y == 0 and self.z == 0 and self.pitch == 0 and self.yaw == 0:
            if self.imu_data["pitch"] is not None and self.imu_data["yaw"] is not None:
                # rospy.loginfo("ROV at rest: Stabilizing yaw and pitch")
                self.stabilizeAtRest()
        
        elif self.fix_heading:
            # rospy.loginfo("ROV moving: Stabilizing heading")
            self.fixHeading()
        elif self.fix_heave:
            # rospy.loginfo("ROV moving: Stabilizing depth")
            self.fixHeave()
        
        # Default: Use joystick input directly
        else:
            # rospy.loginfo("ROV in manual control: Using joystick input")
            Navigation.navigate(self.x, self.y, self.pitch, self.z, self.yaw)

if __name__ == "__main__":
    try:
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Navigation Node...")

