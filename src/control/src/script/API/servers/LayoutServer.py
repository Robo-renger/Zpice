#!/usr/bin/env python3
import rospy
from control.srv import GetLayout, SetLayout
from API.services.LayoutService import LayoutService

class LayoutServer:
    
    def __init__(self):
        rospy.init_node('layouts_server')
        self.service = LayoutService()     
        
    def Layout_service_server(self):
        s1 = rospy.Service('getLayoutService', GetLayout, self.service.handleGetLayout)
        s2 = rospy.Service('setLayoutService', SetLayout, self.service.handleSetLayout)
        rospy.loginfo("Layout Service Server Ready")
        rospy.spin()
        
if __name__ == "__main__":
    LayoutServer().Layout_service_server()