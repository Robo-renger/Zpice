#!/usr/bin/env python3

from zope.interface import implementer, Interface
from interface.PWMDriver import PWMDriver

@implementer(PWMDriver)
class IServo360(Interface):
    def goForward() -> None:
        """
        Makes the servo move clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
    def goBackwards() -> None:
        """
        Makes the servo move counter clockwise with a very small angle
        :param channel: the channel the servo is connected to.
        """
    def Stop() -> None:
        """
        Makes the servo stop the motion
        :param channel: the channel the servo is connected to.
        """
    