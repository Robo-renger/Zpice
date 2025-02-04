#!/usr/bin/env python3
import rospy
from zope.interface import implementer
from control.msg import Joystick
from interface.JoystickMock import IJoystickMock

@implementer(IJoystickMock)
class AutonomousJoystickMock():
    def __init__(self):
        self.pub = rospy.Publisher("/joystick", Joystick, queue_size=10)
        self.data = Joystick()

    def publish(self):
        self.pub.publish(self.data)

    def set_axis(self, axis: str, value: float):
        """
        Set the value of the axis
        """
        try:
            axis_name = f"{axis}_axis"
            if (hasattr(self.data, axis_name)):
                setattr(self.data, axis_name, value)
            else:
                raise ValueError(f"{axis_name} not found")
        except Exception as e:
            rospy.logerr(e)
    
    def set_button(self, button: str, state: bool):
        """
        Set the state of the button
        """
        try:
            button_name = f"button{button}"
            if (hasattr(self.data, button_name)):
                setattr(self.data, button_name, state)
            else:
                raise ValueError(f"{button_name} not found")
        except Exception as e:
            rospy.logerr(e)
