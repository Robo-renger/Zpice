#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
# from std_msgs.msg import ColorRGBA
from services.LEDDriver import LEDDriver
import time

class LEDDriverNode:
    def __init__(self):
        rospy.init_node('led_driver_node', anonymous=False)
        self.led_driver = LEDDriver()
        # print("ana alos")
    def directionCallback(self, msg):
        dir = msg.data
        color_map = {
            "Left": (239, 0, 255),  # PinkPurple
            "Right": (153, 255, 204),  # lightGreen
            "Backward": (239, 255, 0),   # Yellow
            "Forward": (0, 0, 255),  # Blue
            "Down": (255, 0, 0),     # Red
            "Up": (0, 255, 0),       # Green
            "PitchDown": (0, 230, 255),# Cyan
            "PitchUp": (205, 0, 255), # Purple
            "YawRight": (239, 0, 255),  # PinkPurple
            "YawLeft": (153, 255, 204),  # lightGreen
            "Rest": (255, 30, 0) #ORANGE ROBOTECHAWYYY
        }

        if dir in color_map:
            self.led_driver.setAllColors(color_map[dir])

    def run(self):
        rospy.Subscriber("Direction", String, self.directionCallback)


if __name__ == '__main__':
    try:
        node = LEDDriverNode()
        node.run()
        node.led_driver.setAllColors((255,0,0))
        # print("8AMZA")
        # node.led_driver.setBrightness(0.2)
        # node.led_driver.clear()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass