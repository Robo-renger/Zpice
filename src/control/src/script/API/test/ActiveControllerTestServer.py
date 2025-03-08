#!/usr/bin/env python3
import rospy
from  control.srv import GetActiveController, GetActiveControllerResponse

class ActiveControllerTestServer:
    def __init__(self):
        rospy.init_node('active_controller_test_server')

    def handleActiveController(self, req):
        return GetActiveControllerResponse("ps4")
    
    def run(self):
        rospy.Service('get_active_controller', GetActiveController, self.handleActiveController)
        rospy.loginfo("Active Controller Service Server Ready")
        rospy.spin()

if __name__ == "__main__":
    ActiveControllerTestServer().run()

    