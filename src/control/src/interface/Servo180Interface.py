#!/usr/bin/env python3

from zope.interface import implementer, Interface
from interface.PWMDriver import PWMDriver

@implementer(PWMDriver)
class IServo180(Interface):
    def move(step: int) -> None:
        """
        Controls the movment of the servo by specific step up or down
        :param: step: Step to move the servo by (positive or negative).
        """
    def keepInBounds(value: int) -> int:
        """
        Clamp the value within the allowable range.
        """ 
    def setAngle(angle: int):
        """
        Moves the servo to a specific angle
        :param: angle: Desired angle
        """