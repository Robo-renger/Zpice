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
                print(self.joystick.getAxis())
                if self.joystick.isClicked("DCRIGHTGRIPPER_RIGHT"):
                    rospy.loginfo("DCRIGHTGRIPPER_RIGHT button is clicked! TEST NODE TWO")
                else:
                    rospy.loginfo("DCRIGHTGRIPPER_RIGHT button is not clicked. TEST NODE TWO")
                # rospy.sleep(0.3)
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    node = JoystickTest()
    node.run()