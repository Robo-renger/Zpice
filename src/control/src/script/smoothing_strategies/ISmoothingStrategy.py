#!/usr/bin/env python3

from zope.interface import Interface

class ISmoothingStrategy(Interface):
    
    def smooth(current_value: int, target_value: int) -> int:
        """
        Smooth the PWM signal to the motor controller.
        Used by drive method.
        """
        pass
    
    