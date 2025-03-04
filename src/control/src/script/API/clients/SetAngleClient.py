#!/usr/bin/env python3

import rospy
import actionlib
from control.msg import SetAngleAction, SetAngleGoal
from utils.JsonFileHandler import JSONFileHandler

class SetAngleClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('set_angle', SetAngleAction)
        self.goal = SetAngleGoal()

    def feedbackCallback(self, feedback):
        rospy.loginfo(f"Feedback: {feedback}")
    
    def run(self):
        while not rospy.is_shutdown():
            self.goal.angle = JSONFileHandler().fetchData(JSONFileHandler.ANGLEFIXATION)['angle']
            self.client.wait_for_server()
            rospy.loginfo(f"Setting Angle to {self.goal.angle}")
            self.client.send_goal(self.goal, feedback_cb=self.feedbackCallback)
            self.client.wait_for_result()
            rospy.loginfo(f"Result in client{self.client.get_result()}")

if __name__ == "__main__":
    try:
        rospy.init_node('set_Angle_client') 
        client = SetAngleClient()
        client.run()
    except KeyboardInterrupt:   
        print("Shutting down")
    except rospy.ROSInterruptException:
        print("ROS node terminated.")