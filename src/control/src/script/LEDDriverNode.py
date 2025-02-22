#!/usr/bin/env python3

import rospy
# from std_msgs.msg import ColorRGBA
from services.LEDDriver import LEDDriver
import time

class LEDDriverNode:
    def __init__(self):
        rospy.init_node('led_driver_node', anonymous=False)
        self.led_driver = LEDDriver()
        # print("ana alos")
if __name__ == '__main__':
    try:
        node = LEDDriverNode()
        # node.led_driver.setAllColors((255,0,0))
        # time.sleep(3)
        # node.led_driver.clear()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass