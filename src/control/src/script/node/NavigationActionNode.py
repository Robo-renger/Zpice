#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import String
from control.msg import IMU, Depth
from control.msg import SetDepthAction, SetDepthGoal, SetDepthResult, SetDepthFeedback, SetAngleAction, SetAngleGoal, SetAngleResult, SetAngleFeedback
from control.msg import PhotosphereAction, PhotosphereGoal, PhotosphereFeedback, PhotosphereResult

class NavigationActionNode:
    def __init__(self):
        self.yaw = None
        self.depth = None

        self.fixation_pub = rospy.Publisher("/set_fixation", String, queue_size=10)
        self.rotation_pub = rospy.Publisher("/set_rotation", String, queue_size=10)
        rospy.Subscriber("IMU", IMU, self._imuCallback)
        rospy.Subscriber("depth", Depth, self._depthCallback)

        self.depth_action_server = actionlib.SimpleActionServer('set_depth', SetDepthAction, self._fixDepth, False)
        self.depth_action_server.start()
        self.depth_result = SetDepthResult()
        self.depth_feedback = SetDepthFeedback()

        self.angle_action_server = actionlib.SimpleActionServer('set_angle', SetAngleAction, self._fixAngle, False)
        self.angle_action_server.start()
        self.angle_result = SetAngleResult()
        self.angle_feedback = SetAngleFeedback()

        self.photosphere_action_server = actionlib.SimpleActionServer('photosphere', PhotosphereAction, self._photosphereCallback, False)
        self.photosphere_action_server.start()
        self.photosphere_result = PhotosphereResult()
        self.photosphere_feedback = PhotosphereFeedback()

    def _imuCallback(self, msg: IMU):
        self.yaw = msg.yaw

    def _depthCallback(self, msg: Depth):
        self.depth = msg.depth

    def _fixDepth(self, goal: SetDepthGoal):
        if self.depth is None:
            rospy.logwarn("Depth data unavailable")
            self.depth_action_server.set_aborted()
            return
        
        self.fixation_pub.publish("heave")
        
        while not rospy.is_shutdown():
            if self.depth_action_server.is_preempt_requested():
                rospy.loginfo("Preempted depth fixing action.")
                self.fixation_pub.publish("heave_fixed")
                self.depth_action_server.set_preempted()
                return

            if abs(self.depth - goal.depth) > 0.5:
                self.depth_feedback.current_depth = self.depth
                self.depth_action_server.publish_feedback(self.depth_feedback)
                rospy.loginfo(f"Current depth: {self.depth} (Goal: {goal.depth}, Error: {abs(self.depth - goal.depth)})")
            else:
                self.fixation_pub.publish("heave_fixed")
                self.depth_result.success = True
                rospy.loginfo("Depth goal achieved; sending success result.")
                self.depth_action_server.set_succeeded(self.depth_result)
                return

    def _fixAngle(self, goal: SetAngleGoal):
        if self.yaw is None:
            rospy.logwarn("IMU data unavailable")
            self.angle_action_server.set_aborted()
            return
        
        self.fixation_pub.publish("heading")

        while not rospy.is_shutdown():
            if self.angle_action_server.is_preempt_requested():
                rospy.loginfo("Preempted angle fixing action.")
                self.fixation_pub.publish("heading_fixed")
                self.angle_action_server.set_preempted()
                return

            if abs(self.yaw - goal.angle) > 0.5:
                self.angle_feedback.current_angle = self.yaw
                self.angle_action_server.publish_feedback(self.angle_feedback)
                rospy.loginfo(f"Current angle: {self.yaw} (Goal: {goal.angle}, Error: {abs(self.yaw - goal.angle)})")
            else:
                self.fixation_pub.publish("heading_fixed")
                self.angle_result.success = True
                rospy.loginfo("Angle goal achieved; sending success result.")
                self.angle_action_server.set_succeeded(self.angle_result)
                return

    def _photosphereCallback(self, goal: PhotosphereGoal):
        if self.yaw is None or self.depth is None:
            rospy.logwarn("IMU or depth data unavailable")
            self.photosphere_action_server.set_aborted()
            return

        self.rotation_pub.publish("rotate")

        start_yaw = self.yaw
        previous_yaw = start_yaw

        feedback_increment = 10
        current_increment = 0
        rotation_goal = goal.goal

        rospy.loginfo(f"Starting photosphere action at yaw: {start_yaw} degrees, goal: {rotation_goal} degrees")

        while not rospy.is_shutdown():
            if self.photosphere_action_server.is_preempt_requested():
                rospy.loginfo("Photosphere action preempted.")
                self.rotation_pub.publish("stop")
                self.photosphere_action_server.set_preempted()
                return

            current_yaw = self.yaw
            delta_yaw = current_yaw - previous_yaw

            if delta_yaw > 180:
                delta_yaw -= 360
            elif delta_yaw < -180:
                delta_yaw += 360

            if abs(delta_yaw) >= feedback_increment:
                current_increment += feedback_increment
                previous_yaw = current_yaw
                self.photosphere_feedback.incrementer = current_increment
                self.photosphere_action_server.publish_feedback(self.photosphere_feedback)
                rospy.loginfo(f"Feedback: Increment = {current_increment} degrees")

            if current_increment >= (rotation_goal + 10):
                rospy.loginfo("Photosphere action completed.")
                self.rotation_pub.publish("stop")
                self.photosphere_result.success = True
                self.photosphere_action_server.set_succeeded(self.photosphere_result)
                return

if __name__ == "__main__":
    rospy.init_node("navigation_action_node")
    try:
        node = NavigationActionNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Action Server Node...")