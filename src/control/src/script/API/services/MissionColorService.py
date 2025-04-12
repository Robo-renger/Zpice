#!/usr/bin/env python3
import rospy
from control.srv import SetMissionColorResponse
# from services.LEDDriver import LEDDriver
import ast

class MissionColorService:
    def __init__(self):
        # self.led_driver = LEDDriver()
        pass

    def handleSetColor(self, req):
        rospy.loginfo(f"Recieved setcolor request: {req.color}")
        color_tuple = ast.literal_eval(req.color)
        # self.led_driver.setAllColors(color_tuple)
        return SetMissionColorResponse(True)
