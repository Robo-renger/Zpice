#!/usr/bin/env python3
import rospy
from services.Joystick import CJoystick

class TestNodeTwo:
    def __init__(self):
        rospy.init_node('test_node', anonymous=True)
        self.joystick = CJoystick()

    def run(self):
        try:
            while not rospy.is_shutdown():
                x_left, y_left, x_right, y_right = self.joystick.getAxis() 
                rospy.loginfo(f"x_left = {x_left}")
                rospy.loginfo(f"y_left = {y_left}")
                rospy.loginfo(f"x_right = {x_right}")
                rospy.loginfo(f"y_right = {y_right}")
                # if self.joystick.isClicked("RIGHTGRIPPER"):
                #     rospy.loginfo("RIGHTGRIPPER button is clicked! TEST NODE TWO")
                # else:
                #     rospy.loginfo("RIGHTGRIPPER button is not clicked. TEST NODE TWO")
                rospy.sleep(1)
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    node = TestNodeTwo()
    node.run()
