#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
# from std_msgs.msg import ColorRGBA
from services.LEDDriver import LEDDriver
from smoothing_strategies.ExponentialSmoothing import ExponentialSmoothing
import time

class LEDDriverNode:
    def __init__(self):
        rospy.init_node('led_driver_node', anonymous=False)
        self.led_driver = LEDDriver()

        self.smoother_r = ExponentialSmoothing(alpha=0.1)
        self.smoother_g = ExponentialSmoothing(alpha=0.1)
        self.smoother_b = ExponentialSmoothing(alpha=0.1)

        self.en_smoothing = True

        self.current_color = (255, 30, 0)

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
            target_r, target_g, target_b = color_map[dir]
            current_r, current_g, current_b = self.current_color
            if self.en_smoothing:
                smoothed_r = self.smoother_r.smooth(current_r, target_r)
                smoothed_g = self.smoother_g.smooth(current_g, target_g)
                smoothed_b = self.smoother_b.smooth(current_b, target_b)

                finalColor = (smoothed_r, smoothed_g, smoothed_b)
                
            else:
                finalColor = color_map[dir]
            self.led_driver.setAllColors(finalColor)

            self.current_color = finalColor
            

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