#!/usr/bin/env python3
import rospy
from services.Joystick import CJoystick
from services.Switching import Switching

class SwitchingNode:
    def __init__(self) -> None:
        rospy.init_node("switching_node", anonymous=False)
        self.joystick = CJoystick()
        self.switchablePins = {
            'RIGHTGRIPPER': 5,
            'LEFTGRIPPER': 6,
            'FLASH': 26
        }
    def run(self):
        try:
            while not rospy.is_shutdown():
                self.__flash()
                self.__rightGripper()
                self.__leftGripper()
        except Exception as e:
            rospy.logerr(f"Error in DCNode: {e}")
        finally:
            self.joystick.cleanup()

    def __flash(self):
        componentName = 'FLASH'
        switchablePin = self.switchablePins[componentName]
        flashSwitch = Switching(switchablePin) 
        
        if self.joystick.isClicked(componentName):
            print(componentName)
            flashSwitch.open()
            
    def __rightGripper(self):
        componentName = 'RIGHTGRIPPER'
        switchablePin = self.switchablePins[componentName]
        rightGripper = Switching(switchablePin)
        
        if self.joystick.isClicked(componentName):
            print(f"{componentName} TOGGLE")
            rightGripper.toggle()

    def __leftGripper(self):
        componentName = 'LEFTGRIPPER'
        switchablePin = self.switchablePins[componentName]
        leftGripper = Switching(switchablePin)
        
        if self.joystick.isClicked(componentName):
            print(f"{componentName} TOGGLE")
            leftGripper.open()
        

if __name__ == "__main__":
    try:
        left_gripper = SwitchingNode()
        left_gripper.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Switching Node...")
        
    

