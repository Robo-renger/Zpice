#!/usr/bin/env python3
import rospy
import random
from helpers.AutonomousJoystick import AutonomousJoystickMock

class AutonomousJoystickNode:
    def __init__(self):
        rospy.init_node('joystick_node', anonymous=True)
        self.joystick = AutonomousJoystickMock()
        self.rate = rospy.Rate(10)
        self.axis_names = ["x", "y", "z", "pitch", "yaw"]
        self.button_count = 12

    def rand_axes(self):
         for axis in self.axis_names:
                new_value = random.uniform(-1, 1)
                self.joystick.set_axis(axis, new_value)

    def rand_buttons(self):
        for btn in range(1, self.button_count + 1):
                new_state = random.choice([True, False])
                self.joystick.set_button(str(btn), new_state)

    def run(self):
        while not rospy.is_shutdown():
            self.rand_axes()
            self.rand_buttons()
            self.joystick.publish()
            self.rate.sleep()

    
if __name__ == "__main__":
    try:
        node = AutonomousJoystickNode()
        node.run()
    except KeyboardInterrupt:
        print("Shutting down")
    