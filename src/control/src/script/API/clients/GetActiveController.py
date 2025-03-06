#!/usr/bin/env python3
import rospy
from control.srv import GetActiveController

class ActiveController():
    
    def getType(self):
        rospy.wait_for_service('get_active_controller')
        get_active_controller = rospy.ServiceProxy('get_active_controller', GetActiveController)
        return get_active_controller().activeController