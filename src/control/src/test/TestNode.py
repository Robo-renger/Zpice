#!/usr/bin/env python3
import rospy
from script.services.Joystick import CJoystick

class TestNode:
    def __init__(self):
        rospy.init_node('test_node', anonymous=True)
        self.joystick = CJoystick()

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)  # Simulate periodic checking
            try:
                if self.joystick.isClicked("LEFTGRIPPER"):
                    rospy.loginfo("LEFTGRIPPER button is clicked!")
                else:
                    rospy.loginfo("LEFTGRIPPER button is not clicked.")
            except ValueError as e:
                rospy.logwarn(f"Error: {e}")

if __name__ == "__main__":
    node = TestNode()
    node.run()
