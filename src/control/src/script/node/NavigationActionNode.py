#!/usr/bin/env python3

import rospy
import actionlib
import cv2
from gui.src.script.services.FishEyeCamera import FishEyeCamera
from control.msg import IMU, Depth, SetTarget
from control.msg import SetDepthAction, SetDepthGoal, SetDepthResult, SetDepthFeedback, SetAngleAction, SetAngleGoal, SetAngleResult, SetAngleFeedback
from control.msg import PhotosphereAction, PhotosphereGoal, PhotosphereFeedback, PhotosphereResult

class NavigationActionNode:
    def __init__(self):
        self.yaw = None
        self.depth = None
        self.camera = FishEyeCamera()
        self.camera.setupCamera()

        self.set_target_pub = rospy.Publisher("set_target", SetTarget, queue_size=10)
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
        
        msg = SetTarget()
        msg.type = "heave"
        msg.reached = False
        msg.target = goal.depth
        self.set_target_pub.publish(msg)
        
        while not rospy.is_shutdown():
            if self.depth_action_server.is_preempt_requested():
                rospy.loginfo("Preempted depth fixing action.")
                msg.reached = True
                self.set_target_pub.publish(msg)
                self.depth_action_server.set_preempted()
                return

            if abs(self.depth - goal.depth) > 2:
                self.depth_feedback.current_depth = self.depth
                self.depth_action_server.publish_feedback(self.depth_feedback)
                rospy.loginfo(f"Current depth: {self.depth} (Goal: {goal.depth}, Error: {abs(self.depth - goal.depth)})")
            else:
                msg.reached = True
                self.set_target_pub.publish(msg)
                self.depth_result.success = True
                rospy.loginfo("Depth goal achieved; sending success result.")
                self.depth_action_server.set_succeeded(self.depth_result)
                return

    def _fixAngle(self, goal: SetAngleGoal):
        if self.yaw is None:
            rospy.logwarn("IMU data unavailable")
            self.angle_action_server.set_aborted()
            return
        
        msg = SetTarget()
        msg.type = "heading"
        msg.reached = False
        msg.target = goal.angle
        self.set_target_pub.publish(msg)

        while not rospy.is_shutdown():
            if self.angle_action_server.is_preempt_requested():
                rospy.loginfo("Preempted angle fixing action.")
                msg.reached = True
                self.set_target_pub.publish(msg)
                self.angle_action_server.set_preempted()
                return

            if abs(self.yaw - goal.angle) > 5:
                self.angle_feedback.current_angle = self.yaw
                self.angle_action_server.publish_feedback(self.angle_feedback)
                rospy.loginfo(f"Current angle: {self.yaw} (Goal: {goal.angle}, Error: {abs(self.yaw - goal.angle)})")
            else:
                msg.reached = True
                self.set_target_pub.publish(msg)
                self.angle_result.success = True
                rospy.loginfo("Angle goal achieved; sending success result.")
                self.angle_action_server.set_succeeded(self.angle_result)
                return

    def _photosphereCallback(self, goal: PhotosphereGoal):
        if self.yaw is None or self.depth is None:
            rospy.logwarn("IMU or depth data unavailable")
            self.photosphere_action_server.set_aborted()
            return

        rotating_msg = SetTarget()
        rotating_msg.type = "rotate"
        rotating_msg.reached = False
        rotating_msg.target = 360
        self.set_target_pub.publish(rotating_msg)

        paths = []
        index = 0
        start_yaw = self.yaw
        previous_yaw = start_yaw
        feedback_increment = 20
        current_increment = 0
        rotation_goal = 360
        rospy.loginfo(f"Starting photosphere action at yaw: {start_yaw} degrees, goal: {rotation_goal} degrees")

        while not rospy.is_shutdown():
            if self.photosphere_action_server.is_preempt_requested():
                rospy.loginfo("Photosphere action preempted.")
                rotating_msg.reached = True
                self.set_target_pub.publish(rotating_msg)
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
                ret, frame = self.camera.capture.read()
                if ret:
                    screenshot_num = current_increment // feedback_increment
                    screenshot_path = f"/var/www/html/photosphere_mission/photosphere{screenshot_num:02d}.jpg"
                    cv2.imwrite(screenshot_path)
                    paths.append(screenshot_path)
                    rospy.loginfo(f"Saved: {screenshot_path}")
                else:
                    rospy.logwarn("Failed to capture frame")
                index += 1
                rospy.loginfo(f"Feedback: Increment = {current_increment} degrees")

            if current_increment >= (rotation_goal + 10):
                rospy.loginfo("Photosphere action completed.")
                rotating_msg.reached = True
                self.set_target_pub.publish(rotating_msg)
                self.photosphere_result.screenshots = paths
                self.photosphere_action_server.set_succeeded(self.photosphere_result)
                return

if __name__ == "__main__":
    rospy.init_node("navigation_action_node")
    try:
        node = NavigationActionNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Action Server Node...")