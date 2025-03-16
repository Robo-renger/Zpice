#!/usr/bin/env python3
import rospy
from API.services.InitializationService import InitializationService
from control.srv import InitDepth, InitHeading
class InitializationServer:
    def __init__(self):
        rospy.init_node('initialization_server')
        self.service = InitializationService()

    def initialization_service_server(self):
        s1 = rospy.Service('initDepth', InitDepth, self.service.handleInitDepth)
        s2 = rospy.Service('initHeading', InitHeading, self.service.handleInitHeading)
        rospy.loginfo("Initialization Service Server Ready")
        rospy.spin()

if __name__ == "__main__":
    InitializationServer().initialization_service_server()