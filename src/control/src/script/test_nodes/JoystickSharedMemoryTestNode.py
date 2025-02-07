#!/usr/bin/env python3
import rospy
from services.Joystick import CJoystick

class JoystickSharedMemoryTestNode:
    def __init__(self):
        rospy.init_node('joystick_tester_node', anonymous=True)
        self.joystick = CJoystick()

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.joystick.isClicked("RIGHTGRIPPER"):
                    rospy.loginfo("RIGHTGRIPPER button is clicked! TEST NODE TWO")
                else:
                    rospy.loginfo("RIGHTGRIPPER button is not clicked. TEST NODE TWO")
                rospy.sleep(1)
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    node = JoystickSharedMemoryTestNode()
    node.run()