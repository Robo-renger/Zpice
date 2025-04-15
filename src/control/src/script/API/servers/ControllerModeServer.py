#!/usr/bin/env python3
import rospy
from API.services.ControllerModeService import ControllerModeService
from control.srv import SwitchJoystickMode
class ControllerModeServer:
    def __init__(self):
        rospy.init_node('controller_mode_server')
        self.service = ControllerModeService()

    def controller_service_server(self):
        rospy.Service('switchJoystickMode', SwitchJoystickMode, self.service.switchMode)
        rospy.loginfo("Controller Mode Service Server Ready")
        rospy.spin()

if __name__ == "__main__":
    ControllerModeServer().controller_service_server()