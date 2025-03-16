#!/usr/bin/env python3
import rospy
from control.srv import InitDepthResponse, InitHeadingResponse

class InitializationService:
    def __init__(self):
        pass

    def handleInitDepth(self, req):
        rospy.loginfo(f"Initialization depth is: {req.depth}")
        initialization_depth = req.depth
        return InitDepthResponse(True)
    
    def handleInitHeading(self, req):
        rospy.loginfo(f"Initialization heading is: {req.heading}")
        initialization_heading = req.heading
        return InitHeadingResponse(True)