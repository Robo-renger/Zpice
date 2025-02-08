#!/usr/bin/env python3

import rospy
from services.Switching import Switching
from services.Joystick import CJoystick

class SwitchingNode:
    def __init__(self, pin: int, button_name: str) -> None:
        rospy.init_node('switching_node', anonymous=False)
        self.switching = Switching(pin)
        self.button_name = button_name
        self.joystick = CJoystick()

    def switch(self) -> None:
        try:
            while not rospy.is_shutdown():
                if self.joystick.isClicked(self.button_name):
                    self.switching.toggle()
                    rospy.loginfo(f"{self.button_name.lower()} button is clicked! TEST NODE TWO")
                else:
                    rospy.loginfo(f"{self.button_name.lower()} button is not clicked. TEST NODE TWO")
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    try:
        flash = SwitchingNode(12, "FLASH")
        flash.switch()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")


    

    
        