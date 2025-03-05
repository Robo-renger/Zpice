#!/usr/bin/env python3
import rospy
from services.Joystick import CJoystick
from utils.Configurator import Configurator
from utils.LayoutManager import LayoutManager
from std_msgs.msg import String

class JoystickModeSwitcher:
    def __init__(self):
        rospy.init_node('joystick_mode_switcher')
        self.pub = rospy.Publisher('/joystick_current_mode', String, queue_size=10)
        self.joystick = CJoystick()
        self.configurator = Configurator()
        self.joystick_layout = LayoutManager().fetchLayout("controller")
        self.current_mode_index = 0
        self.num_modes = len(self.joystick_layout["ps4"]["modes"])

    def run(self):
        while not rospy.is_shutdown():
            if self.joystick.isPressed('RIGHTGRIPPER') and self.joystick.isPressed('LEFTGRIPPER_OPEN'):
                self.current_mode_index = (self.current_mode_index + 1) % self.num_modes
                new_mode = self.joystick_layout["ps4"]["modes"][self.current_mode_index]
                self.configurator.setConfig("joystick_buttons", new_mode)
            self.pub.publish(self.current_mode_index)

if __name__ == "__main__":
    try:
        switcher = JoystickModeSwitcher()
        switcher.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
    except rospy.ROSInterruptException as e:
        rospy.loginfo(f"Error in swiching joystick mode: {e}")