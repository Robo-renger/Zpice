#!/usr/bin/env python3

import rospy
import actionlib
from control.msg import SetDepthAction, SetDepthGoal
from utils.JsonFileHandler import JSONFileHandler

class SetDepthClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('set_depth', SetDepthAction)
        self.goal = SetDepthGoal()

    def feedbackCallback(self, feedback):
        rospy.loginfo(f"Feedback: {feedback}")
    
    def run(self):
        while not rospy.is_shutdown():
            self.goal.depth = JSONFileHandler().fetchData(JSONFileHandler.DEPTHFIXATION)['depth']
            self.client.wait_for_server()
            rospy.loginfo(f"Setting depth to {self.goal.depth}")
            self.client.send_goal(self.goal, feedback_cb=self.feedbackCallback)
            self.client.wait_for_result()
            rospy.loginfo(f"Result in client{self.client.get_result()}")

if __name__ == "__main__":
    try:
        rospy.init_node('set_depth_test_client') 
        client = SetDepthClient()
        client.run()
    except KeyboardInterrupt:   
        print("Shutting down")
    except rospy.ROSInterruptException:
        print("ROS node terminated.")