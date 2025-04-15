#!/usr/bin/env python3
import rospy
from control.srv import SetSpeedPercentageResponse

class SpeedPercentageService:
    def __init__(self):
        pass

    def handleSetSpeedPercentage(self, req):
        rospy.loginfo(f"Speed Percentage is: {req.speedPercentage}")
        speed_percentage = req.speedPercentage
        return SetSpeedPercentageResponse(True)
    