#!/usr/bin/env python3
import rospy
from gui.srv import screenshots
from API.services.screenshotsService import ScreenshotsService

class screenshotsServer:
    
    def __init__(self):
        rospy.init_node('screenshots_server')
        self.service = ScreenshotsService()
        
    def screenshots_service_server(self):
        s1 = rospy.Service('screenshotsService', screenshots, self.service.handleGetScreenshots)
        rospy.loginfo("screenshots Service Server Ready")
        rospy.spin()
if __name__ == "__main__":
    screenshotsServer().screenshots_service_server()