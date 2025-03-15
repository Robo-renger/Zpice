#!/usr/bin/env python3

import rospy
import actionlib
from control.msg import IMU, Depth
from control.msg import SetDepthAction, SetDepthGoal, SetDepthResult, SetDepthFeedback, SetAngleAction, SetAngleGoal, SetAngleResult, SetAngleFeedback
from std_msgs.msg import String
from services.Joystick import CJoystick
from services.Navigation import Navigation
from services.Vectorizer import Vectorizer
from services.PIDController import PIDController
from utils.Configurator import Configurator
import time

class NavigationNode:
    def __init__(self):
        self.joystick = CJoystick()
        self.x = 0
        self.y = 0
        self.z = 0
        self.pitch = 0
        self.yaw = 0
        self.last_reset_time = 0  
        self.PID_configs = Configurator().fetchData(Configurator.PID_PARAMS)
        
        self.pid_yaw   = PIDController(self.PID_configs['yaw_KP'], self.PID_configs['yaw_KI'], self.PID_configs['yaw_KD'])
        self.pid_pitch = PIDController(self.PID_configs['pitch_KP'], self.PID_configs['pitch_KI'], self.PID_configs['pitch_KD'])
        self.pid_heave = PIDController(self.PID_configs['heave_KP'], self.PID_configs['heave_KI'], self.PID_configs['heave_KD'])
        self.imu_data = {
            'pitch': None,
            'yaw' : None
        }
        self.depth = None

        self.fix_heading = False
        self.fix_heave = False
        
        rospy.Subscriber("IMU", IMU, self._imuCallback)
        rospy.Subscriber("depth", Depth, self._depthCallback)
        # rospy.Subscriber("/set", String, self._setFixationCallback)

        self.depth_action_server = actionlib.SimpleActionServer('set_depth', SetDepthAction, self._fixDepth, False)
        self.depth_action_server.start()
        self.depth_result = SetDepthResult()
        self.depth_feedback = SetDepthFeedback()

        self.angle_action_server = actionlib.SimpleActionServer('set_angle', SetAngleAction, self._fixAngle, False)
        self.angle_action_server.start()
        self.angle_result = SetAngleResult()
        self.angle_feedback = SetAngleFeedback()
     
    def reload(self):
        self.PID_configs = Configurator().fetchData(Configurator.PID_PARAMS)
        self.pid_yaw   = PIDController(self.PID_configs['yaw_KP'], self.PID_configs['yaw_KI'], self.PID_configs['yaw_KD'])
        self.pid_pitch = PIDController(self.PID_configs['pitch_KP'], self.PID_configs['pitch_KI'], self.PID_configs['pitch_KD'])
        self.pid_heave = PIDController(self.PID_configs['heave_KP'], self.PID_configs['heave_KI'], self.PID_configs['heave_KD'])
        
    def _imuCallback(self, msg: IMU):
        self.imu_data['pitch'] = msg.pitch
        self.imu_data['yaw'] = msg.yaw

    def _depthCallback(self, msg: Depth):
        self.depth = msg.depth

    def _fixDepth(self, goal: SetDepthGoal):
        self.fix_heave = True
        if self.depth is None:
            rospy.logwarn("Depth data unavailable")
            self.depth_action_server.set_aborted()
            return

        while not rospy.is_shutdown():
            if self.depth_action_server.is_preempt_requested():
                rospy.loginfo("Preempted depth fixing action.")
                self.depth_action_server.set_preempted()
                return

            if abs(self.depth - goal.depth) > 0.5:
                self.depth_feedback.current_depth = self.depth
                self.depth_action_server.publish_feedback(self.depth_feedback)
                rospy.loginfo(f"Current depth: {self.depth} (Goal: {goal.depth}, Error: {abs(self.depth - goal.depth)})")
            else:
                self.pid_yaw.updateSetpoint(goal.depth)
                self.depth_result.success = True
                rospy.loginfo("Depth goal achieved; sending success result.")
                self.depth_action_server.set_succeeded(self.depth_result)
                return


    def _fixAngle(self, goal: SetAngleGoal):
        self.fix_heading = True
        if self.imu_data['yaw'] is None:
            rospy.logwarn("IMU data unavailable")
            self.angle_action_server.set_aborted()
            return
        
        while not rospy.is_shutdown():
            if self.angle_action_server.is_preempt_requested():
                    rospy.loginfo("Preempted angle fixing action.")
                    self.angle_action_server.set_preempted()
                    return

            if abs(self.imu_data['yaw'] - goal.angle) > 0.5:
                self.angle_feedback.current_angle = self.imu_data['yaw']
                self.angle_action_server.publish_feedback(self.angle_feedback)
                rospy.loginfo(f"Current angle: {self.imu_data['yaw']} (Goal: {goal.angle}, Error: {abs(self.imu_data['yaw'] - goal.angle)})")
            else:
                self.pid_heave.updateSetpoint(goal.angle)
                self.angle_result.success = True
                rospy.loginfo("angle goal achieved; sending success result.")
                self.angle_action_server.set_succeeded(self.angle_result)
                return

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

        if (self.x != 0 or self.y != 0 or self.z != 0 or self.pitch != 0 or self.yaw != 0) and not (self.fix_heading or self.fix_heave):
            self.pid_yaw.updateSetpoint(self.imu_data['yaw'])
            self.pid_pitch.updateSetpoint(self.imu_data['pitch'])

    def stabilizeAtRest(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        pitch_output = self.pid_pitch.stabilize(self.imu_data['pitch'])
        Navigation.navigate(0, 0, -pitch_output, 0, -yaw_output)

    def fixHeave(self):
        depth_output = self.pid_heave.stabilize(self.depth)
        self.depth_result.success = True
        self.depth_action_server.set_succeeded(self.depth_result)
        Navigation.navigate(self.x, self.y, 0, depth_output, self.yaw)

    def fixHeading(self):
        yaw_output = self.pid_yaw.stabilize(self.imu_data['yaw'])
        Navigation.navigate(0, 0, self.pitch, self.z, -yaw_output)

    def navigate(self):        
        # Vectorizer.yaw_only = False
        self.handleJoystickInput()

        # ROV is at rest
        if self.x == 0 and self.y == 0 and self.z == 0 and self.pitch == 0 and self.yaw == 0 and not (self.fix_heading or self.fix_heave):
            if self.imu_data['pitch'] is not None and self.imu_data['yaw'] is not None:
                # rospy.loginfo("ROV at rest: Stabilizing yaw and pitch")
                self.stabilizeAtRest()
        
        elif self.fix_heading and (self.z != 0 or self.pitch != 0):
            # rospy.loginfo("ROV moving: Stabilizing heading")
            self.fixHeading()
        elif self.fix_heave and (self.x != 0 or self.y != 0 or self.yaw != 0):
            # rospy.loginfo("ROV moving: Stabilizing depth")
            self.fixHeave()

        # Default: Use joystick input directly
        else:
            # rospy.loginfo("ROV in manual control: Using joystick input")
            Navigation.navigate(self.x, self.y, self.pitch, self.z, self.yaw)

if __name__ == "__main__":
    rospy.init_node("navigation_node")
    try:
        node = NavigationNode()
        while not rospy.is_shutdown():
            node.navigate()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Navigation Node...")