#!/usr/bin/env python3

import rospy
from gui.msg import Joystick

class JoystickPublisher:
    def __init__(self):
        rospy.init_node('joystick_publisher', anonymous=True)
        self.publisher = rospy.Publisher('/joystick', Joystick, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def publish_joystick_data(self):
        while not rospy.is_shutdown():
            msg = Joystick()
            # Populate the msg fields here
            msg.x_axis = 0.2  # Example field
            msg.y_axis = -0.5  # Example field
            self.publisher.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    publisher = JoystickPublisher()
    publisher.publish_joystick_data()
