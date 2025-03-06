#!/usr/bin/env python3
import rospy
from control.srv import GetActiveController, GetActiveControllerResponse

class SetJoystickTypeTest:
    def __init__(self):
        rospy.init_node('set_joystick_type_test')

    def getType(self, req):
        return GetActiveControllerResponse("ps4")
    
    def run(self):
        rospy.Service('get_active_controller', GetActiveController, self.getType)
        
if __name__ == "__main__":
    try:
        node = SetJoystickTypeTest()
        node.run()
        rospy.spin() 
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
    except rospy.ROSException as e:
        rospy.loginfo(f"Error in the service: {e}")
