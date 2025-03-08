#!/usr/bin/env python3
import rospy
from control.srv import SetMissionColor
from API.services.MissionColorService import MissionColorService

class MissionColorServer:
    def __init__(self):
        rospy.init_node('mission_color_server')
        self.service = MissionColorService()

    def missionColorServer(self):
        rospy.Service('setMissionColor', SetMissionColor, self.service.handleSetColor)
        rospy.loginfo("Mission Color Service Server Ready")
        rospy.spin()
if __name__ == "__main__":
    MissionColorServer().missionColorServer()