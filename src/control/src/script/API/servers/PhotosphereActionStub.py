#!/usr/bin/env python

import rospy
import actionlib
from control.msg import PhotosphereAction, PhotosphereFeedback, PhotosphereResult

class PhotosphereActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('photosphere', PhotosphereAction, self._photosphereCallback, False)
        self.server.start()
        rospy.loginfo("Photosphere Action Server started.")

    def _photosphereCallback(self, goal):
        goal = 360
        feedback = PhotosphereFeedback()
        result = PhotosphereResult()

        current_angle = 0.0
        screenshots = []

        rate = rospy.Rate(1)  # 1 Hz

        while current_angle < goal:
            if self.server.is_preempt_requested():
                rospy.logwarn("Goal preempted")
                self.server.set_preempted()
                return

            current_angle += 45.0  # Simulate some rotation
            feedback.incrementer = current_angle
            feedback.discription = f"Photosphered {current_angle} degrees"
            self.server.publish_feedback(feedback)
            rospy.loginfo(feedback.discription)

            screenshots.append(f"/var/www/html/photosphere_mission/photosphere{current_angle}.jpg")  # Simulated screenshot name
            rate.sleep()

        result.success = True
        result.screenshots = screenshots
        self.server.set_succeeded(result)
        rospy.loginfo("Rotation completed successfully.")

if __name__ == '__main__':
    rospy.init_node('Photosphere_action_server')
    server = PhotosphereActionServer()
    rospy.spin()
