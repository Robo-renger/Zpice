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
            'LEFTGRIPPER_UP_DOWN': self.__pins['LEFTGRIPPER_UP_DOWN_SWITCH'],
            'LEFTGRIPPER': self.__pins['LEFT_GRIPPER_SWITCH'],
            'VERTICALGRIPPER': self.__pins['VERTICALGRIPPER_SWITCH'],
            'SYRINGE': self.__pins['SYRINGE_SWITCH'],
            'FRONTGRIPPER': self.__pins['FRONTGRIPPER_SWITCH']
        }
        # Create Switching instances and store them
        self.switches = {
            component: Switching(pin)
            for component, pin in self.switchablePins.items()
        }

    def run(self):
        try:
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                self.__leftGripperVeticalMechanism()
                self.__verticalGripper()
                self.__leftGripper()
                self.__syringe()
                self.__frontGripper()
                rate.sleep()  # <-- This is the key fix

        except Exception as e:
            rospy.logerr(f"Error in Switching Node: {e}")
        finally:
            self.joystick.cleanup()

    def __leftGripperVeticalMechanism(self):
        componentName = 'LEFTGRIPPER_UP_DOWN'
        mechanismSwitch = self.switches[componentName]

        if self.joystick.isClicked(componentName):
            print(f"{componentName} TOGGLE")
            mechanismSwitch.toggle()

    def __verticalGripper(self):
        componentName = 'VERTICALGRIPPER'
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
    def __frontGripper(self):
        componentName = 'FRONTGRIPPER'
        leftGripper = self.switches[componentName]

        if self.joystick.isClicked(componentName):
            print(f"{componentName} TOGGLE")
            leftGripper.toggle()

    def __syringe(self):
        componentName = 'SYRINGE'
        syringe = self.switches[componentName]

        if self.joystick.isClicked(componentName) == 2:
            print(f"{componentName} TOGGLE")
            syringe.toggle()


if __name__ == "__main__":
    try:
        switching_node = SwitchingNode()
        switching_node.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting Switching Node...")
