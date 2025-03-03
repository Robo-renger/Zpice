#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from services.LEDDriver import LEDDriver
from smoothing_strategies.ExponentialSmoothing import ExponentialSmoothing
import time

class LEDDriverNode:
    def __init__(self):
        rospy.init_node('led_driver_node', anonymous=False)
        self.led_driver = LEDDriver()

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
            self.led_driver.enableSmoothing(True)
            self.led_driver.setAllColors(color_map[dir])
            
    def run(self):
        rospy.Subscriber("Direction", String, self.directionCallback)


if __name__ == '__main__':
    try:
        node = LEDDriverNode()
        node.run()
        node.led_driver.setAllColors((255,0,0))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass