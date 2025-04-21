#!/usr/bin/env python3

import rospy
import actionlib
from control.msg import PhotosphereAction, PhotosphereGoal, PhotosphereFeedback

class PhotosphereClient:
    def _init_(self, server_name="photosphere"):
        # rospy.loginfo(f"Initializing Action")
        self.client = actionlib.SimpleActionClient(server_name, PhotosphereAction)
        rospy.loginfo(f"Waiting for action server '{server_name}'...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to photosphere action server.")

    def sendGoal(self, angle_increment):
        goal = PhotosphereGoal()
        goal.angle = angle_increment
        rospy.loginfo(f"Sending photosphere goal with angle increment: {angle_increment} degrees")
        self.client.send_goal(goal, feedback_cb=self._feedback_cb)

    def _feedback_cb(self, feedback: PhotosphereFeedback):
        rospy.loginfo(f"Feedback: {feedback.incrementer:.2f}% - URL: {feedback.url}")

    def wait_for_result(self):
        rospy.loginfo("Waiting for result...")
        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.loginfo(f"Photosphere completed. Directory: {result.directory}")

if __name__ == "__main__":
    rospy.init_node('photosphere_test_client', anonymous=True)
    try:
        client = PhotosphereClient()
        client.sendGoal(90)
        client.wait_for_result()
    except Exception as e:
        rospy.loginfo(f"Photosphere client interrupted. {e}")