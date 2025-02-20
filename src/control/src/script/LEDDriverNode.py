#!/usr/bin/env python3

import rospy
from std_msgs.msg import ColorRGBA
from services.LEDDriver import LEDDriver
import time

class LEDDriverNode:
    def __init__(self):
        self.led_driver = LEDDriver(pin=27, num_leds=16, brightness=255)
        rospy.init_node('led_driver_node', anonymous=True)
        rospy.Subscriber('/led_command', ColorRGBA, self.led_command_callback)

    def led_command_callback(self, msg):
        r = int(msg.r * 255)
        g = int(msg.g * 255)
        b = int(msg.b * 255)

        self.led_driver.setAllColors((r, g, b))
        rospy.loginfo(f"Set all LEDs to color: R={r}, G={g}, B={b}")

if __name__ == '__main__':
    try:
        node = LEDDriverNode()
        node.led_driver.setAllColors((255,0,0))
        time.sleep(3)
        node.led_driver.clear()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass