#!/usr/bin/env python3
import rospy
from services.Joystick import CJoystick
from services.Switching import Switching
from utils.Configurator import Configurator


class SwitchingNode:
    def __init__(self) -> None:
        rospy.init_node("switching_node", anonymous=False)
        self.joystick = CJoystick()

        self.__pins = Configurator().fetchData(Configurator().PINS)
        self.switchablePins = {
            'RIGHTGRIPPER': self.__pins['RIGHT_GRIPPER_SWITCH'],
            'LEFTGRIPPER': self.__pins['LEFT_GRIPPER_SWITCH'],
            'FLASH': self.__pins['FLASH_SWITCH']
        }
        # Create Switching instances and store them
        self.switches = {
            component: Switching(pin)
            for component, pin in self.switchablePins.items()
        }

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.__flash()
                self.__rightGripper()
                self.__leftGripper()
        except Exception as e:
            rospy.logerr(f"Error in Switching Node: {e}")
        finally:
            self.joystick.cleanup()

    def __flash(self):
        componentName = 'FLASH'
        flashSwitch = self.switches[componentName]

        if self.joystick.isClicked(componentName):
            print(f"is open before: {flashSwitch.opened}")
            flashSwitch.toggle()
            print(f"is open after: {flashSwitch.opened}")

    def __rightGripper(self):
        componentName = 'RIGHTGRIPPER'
        rightGripper = self.switches[componentName]

        if self.joystick.isClicked(componentName):
            print(f"{componentName} TOGGLE")
            rightGripper.toggle()

    def __leftGripper(self):
        componentName = 'LEFTGRIPPER'
        leftGripper = self.switches[componentName]

        if self.joystick.isClicked(componentName):
            print(f"{componentName} TOGGLE")
            leftGripper.toggle()


if __name__ == "__main__":
    try:
        switching_node = SwitchingNode()
        switching_node.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Switching Node...")
