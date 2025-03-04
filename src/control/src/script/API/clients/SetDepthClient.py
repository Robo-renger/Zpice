#!/usr/bin/env python3

import rospy
import actionlib
from control.msg import SetDepthAction, SetDepthGoal, SetDepthResult, SetDepthFeedback
from utils.JsonFileHandler import JSONFileHandler

class SetDepthClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('set_depth', SetDepthAction)
        # self.state = True
        self.client.wait_for_server()
    
    def run(self):
        goal = SetDepthGoal()
        goal.depth = JSONFileHandler().fetchData(JSONFileHandler.DEPTHFIXATION)['depth']
        rospy.loginfo(f"Setting depth to {goal.depth}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo(self.client.get_result())

if __name__ == "__main__":
    try:
        rospy.init_node('set_depth_client')
        client = SetDepthClient()
        client.run()
        rospy.spin()
    except KeyboardInterrupt:   
        print("Shutting down")
    except rospy.ROSInterruptException:
        print("ROS node terminated.")