#!/usr/bin/env python3

import rospy
import actionlib
from control.msg import PhotosphereAction, PhotosphereGoal, PhotosphereFeedback

class PhotosphereClient:
    def _init_(self, server_name="photosphere"):
        self.client = actionlib.SimpleActionClient(server_name, PhotosphereAction)
        rospy.loginfo(f"Waiting for action server '{server_name}'...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to photosphere action server.")

    def send_goal(self, angle_increment):
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
    try:
        rospy.init_node('photosphere_test_client', anonymous=True)
        client = PhotosphereClient()
        client.send_goal(90)
        client.wait_for_result()
    except rospy.ROSInterruptException:
        rospy.loginfo("Photosphere client interrupted.")