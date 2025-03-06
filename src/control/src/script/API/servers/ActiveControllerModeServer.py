#!/usr/bin/env python3
import rospy
from API.services.ActiveControllerModeService import ActiveControllerModeService
from control.srv import GetActiveController, SwitchJoystickMode
class ActiveControllerModeServer:
    def __init__(self):
        rospy.init_node('active_controller_mode_server')
        self.service = ActiveControllerModeService()

    def active_controller_service_server(self):
        s1 = rospy.Service('getActiveController', GetActiveController, self.service.getType)
        s2 = rospy.Service('switchJoystickMode', SwitchJoystickMode, self.service.switchMode)
        rospy.loginfo("Active Controller Mode Service Server Ready")
        rospy.spin()

if __name__ == "__main__":
    ActiveControllerModeServer().active_controller_service_server()