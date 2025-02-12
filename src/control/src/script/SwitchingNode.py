#!/usr/bin/env python3
import rospy
from services.Joystick import CJoystick
from services.Switching import Switching

class SwitchingNode:
    def __init__(self) -> None:
        rospy.init_node("switching_node", anonymous=False)
        self.joystick = CJoystick()
        
    def run(self):
        try:
            while not rospy.is_shutdown():
                self.flash()
                self.rightGripper()
                self.leftGripper()
        except Exception as e:
            rospy.logerr(f"Error in DCNode: {e}")
        finally:
            self.joystick.cleanup()

    def flash(self):
        flashSwitch = Switching(12) 
        if self.joystick.isClicked("FLASH"):
            print("FLASH")
            flashSwitch.open()
        # else:
        #     flashSwitch(26).close()
    def rightGripper(self):
        rightGripper = Switching(5)
        if self.joystick.isClicked("RIGHTGRIPPER_OPEN"):
            print("RIGHT_OPEN")
            rightGripper.open()
        # else:
        #     rightGripper.close()

    def leftGripper(self):
        leftGripper = Switching(6)
        if self.joystick.isClicked("LEFTGRIPPER_OPEN"):
            print("LEFT_OPEN")
            leftGripper.open()
        # else:
        #     leftGripper.close()

if __name__ == "__main__":
    try:
        left_gripper = SwitchingNode()
        left_gripper.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
        
    

