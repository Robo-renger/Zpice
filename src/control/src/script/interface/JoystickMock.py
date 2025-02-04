#!/usr/bin/env python3
import rospy
from zope.interface import Interface, implementer
from control.msg import Joystick

class IJoystickMock(Interface):
    def __init__(self) -> None:
        """Initialize the publisher on joystick topic."""

    def publish(self, data: Joystick) -> None:
        """Publish the joystick data to the topic."""

    def set_axes(self, axis: str, value: float) -> None:
        """Sets the value of the axis"""

    def set_button(self, button: str, state: bool) -> None:
        """Sets the state of the button either pressed or not"""

    def _keepInBounds(self, value: float) -> float:
        """Clamp the value within the allowable range.""" 