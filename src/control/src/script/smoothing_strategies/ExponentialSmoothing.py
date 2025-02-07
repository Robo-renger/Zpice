#!/usr/bin/env python3
import sys
import os

from zope.interface import implementer

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from interface.ISmoothingStrategy import ISmoothingStrategy

@implementer(ISmoothingStrategy)
class ExponentialSmoothing:
    def __init__(self, alpha: float = 0.1):

        if not (0 < alpha < 1):
            raise ValueError("Alpha must be between 0 and 1.")
        self.alpha = alpha

    # def smooth(self, current_value: int, target_value: int) -> int:

    #     smoothed_value = (1 - self.alpha) * current_value + \
    #         self.alpha * target_value
    #     return int(smoothed_value)
    
    def smooth(self, current_value: int, target_value: int, tolerance: int = 20) -> int:

        smoothed_value = (1 - self.alpha) * current_value + self.alpha * target_value

        # If the smoothed value is within the tolerance, snap to the target value
        if abs(smoothed_value - target_value) <= tolerance:
            return target_value

        return int(smoothed_value)
