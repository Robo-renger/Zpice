#!/usr/bin/env python3
import rospy
from utils.Configurator import Configurator
from services.ManualJoystick import ManualJoystickMock
from services.Keyboard import Keyboard

class ManualJoystickNode:
    def __init__(self):
        rospy.init_node('joystick_node', anonymous=True)
        self.joystick = ManualJoystickMock()
        self.keyboard = Keyboard()
        self.configurator = Configurator()
        self.buttons = self.__getJoystickButtons()
        self.axes = self.__getJoystickAxes()
        self.rate = rospy.Rate(10)

    def __getJoystickButtons(self):
        return self.configurator.fetchData(Configurator.KEYBOARD_BUTTONS)

    def __getJoystickAxes(self):
        return self.configurator.fetchData(Configurator.KEYBOARD_AXES) 

    def run(self):
        while not rospy.is_shutdown():
            key = self.keyboard.getKey()
            if key in self.buttons:
                self.joystick.trigger_button(self.buttons[key])
            elif key in self.axes:
                self.joystick.set_axis(self.axes[key]['axis'], 0.05, self.axes[key]['direction'])

            self.joystick.publish()
            self.rate.sleep()

    
if __name__ == "__main__":
    try:
        node = ManualJoystickNode()
        node.run()
    except KeyboardInterrupt:
        print("Shutting down")
    