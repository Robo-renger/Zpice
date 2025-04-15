#!/usr/bin/env python

import rospy
import actionlib
from control.msg import PhotosphereAction, PhotosphereGoal

def feedback_cb(feedback):
    rospy.loginfo(f"[Feedback] {feedback.discription} ({feedback.incrementer}°)")

def photosphere_client():
    rospy.init_node('photosphere_action_client')

    client = actionlib.SimpleActionClient('photosphere', PhotosphereAction)
    rospy.loginfo("Waiting for Photosphere Action Server to start...")
    client.wait_for_server()
    rospy.loginfo("Photosphere Action Server is up. Sending goal...")

    client.send_goal("", feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()
    if result.success:
        rospy.loginfo("Photosphere mission completed successfully.")
        rospy.loginfo(f"Screenshots: {result.screenshots}")
    else:
        rospy.logwarn("Photosphere mission failed or was preempted.")

if __name__ == '__main__':
    try:
        photosphere_client()
    except rospy.ROSInterruptException:
        pass
