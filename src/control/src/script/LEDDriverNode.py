#!/usr/bin/env python3

import rospy
import threading
# from std_msgs.msg import ColorRGBA
from services.LEDDriver import LEDDriver
import time

class LEDDriverNode:
    def __init__(self):
        rospy.init_node('led_driver_node', anonymous=False)
        self.led_driver = LEDDriver()
        print("LED Driverr 3nnnnn 3nnnnnn")

if __name__ == '__main__':
    try:
        node = LEDDriverNode()

        node.led_driver.setAllColors((255,100,50))
        time.sleep(3)
        node.led_driver.clear()
        print("Color set")

        # node.led_driver.setBrightness(0.5)       
        # time.sleep(3)
        # node.led_driver.setBrightness(0.2)
        # time.sleep(3)
        # node.led_driver.setBrightness(1)
        #        
        print("BTE8MEZLAYYYYYYY")
        node.led_driver.blink(7, 0.5, 0.5)
        
        # node.led_driver.rainbowCycle(0.1, 5)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
