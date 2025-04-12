#!/usr/bin/env python3
import rospy
from API.services.SpeedPercentageService import SpeedPercentageService
from control.srv import SetSpeedPercentage
class SpeedPercentageServer:
    def __init__(self):
        rospy.init_node('speed_percentage_server')
        self.service = SpeedPercentageService()

    def speed_percentage_service_server(self):
        s1 = rospy.Service('setSpeedPercentage', SetSpeedPercentage, self.service.handleSetSpeedPercentage)
        rospy.loginfo("Speed Percentage Service Server Ready")
        rospy.spin()

if __name__ == "__main__":
    SpeedPercentageServer().speed_percentage_service_server()