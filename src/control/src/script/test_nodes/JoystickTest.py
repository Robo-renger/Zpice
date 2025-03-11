#!/usr/bin/env python3
import rospy
from services.Joystick import CJoystick

class JoystickTest:
    def __init__(self):
        rospy.init_node('joystick_test_node', anonymous=True)
        self.joystick = CJoystick()

    def run(self):
        try:
            while not rospy.is_shutdown():
                # print(self.joystick.getAxis())
                
                # if self.joystick.isNthClicked("DCRIGHTGRIPPER_RIGHT",2,0.35):
                #     rospy.logwarn("DCRIGHTGRIPPER_RIGHT button is double clicked! TEST NODE TWO")
                
                # if self.joystick.isNthPressed("DCRIGHTGRIPPER_RIGHT",2,0.4):
                #     rospy.logwarn("DCRIGHTGRIPPER_RIGHT button is double Pressed! TEST NODE TWO")
                
                # if self.joystick.isNthClicked("DCRIGHTGRIPPER_RIGHT", 2, 0.3):
                clicks = self.joystick.isPressed("DCRIGHTGRIPPER_RIGHT")
                if clicks == 1:
                    rospy.loginfo("Moving ClockWise")
                elif clicks == 2:
                    rospy.loginfo("Moving CounterClockWise")
                else:
                    rospy.loginfo("Stopped")
                    

                # if self.joystick.isClicked("DCRIGHTGRIPPER_RIGHT"):
                #     rospy.loginfo("DCRIGHTGRIPPER_RIGHT button is clicked! TEST NODE TWO")
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    node = JoystickTest()
    node.run()