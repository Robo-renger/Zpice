#!/usr/bin/env python3
import rospy
from control.srv import StartStopNodeResponse

class JoystickNodeOnOffService:

    def __init__(self):
        pass

    def startNode(self, req):
        self.mapping_name = req.mapping_name
        return StartStopNodeResponse(f"KOLOOOOO BONOOO EL NODE {self.mapping_name} 48ALA")

    def stopNode(self, req):
        self.mapping_name = req.mapping_name
        return StartStopNodeResponse(f"KOLOOOOO BONOOO EL NODE {self.mapping_name} MAYTAAAAA")
