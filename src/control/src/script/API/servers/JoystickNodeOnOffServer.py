#!/usr/bin/env python3
import rospy
from control.srv import StartStopNode
from API.services.JoystickNodeOnOffService import JoystickNodeOnOffService

class JoystickNodeOnOffServer:

    def __init__(self):
        rospy.init_node('joystick_node_on_off_server')
        self.service = JoystickNodeOnOffService()
        
    def joystick_node_service_server(self):
        s1 = rospy.Service('start', StartStopNode, self.service.startNode)
        s2 = rospy.Service('stop', StartStopNode, self.service.stopNode)
        rospy.loginfo("joystick_node Service Server Ready")
        rospy.spin()
if __name__ == "__main__":
    JoystickNodeOnOffServer().joystick_node_service_server()