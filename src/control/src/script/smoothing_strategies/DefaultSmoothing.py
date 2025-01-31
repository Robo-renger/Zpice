from zope.interface import implementer

from interface.iSmoothingStrategy import ISmoothingStrategy


@implementer(ISmoothingStrategy)
class DefaultSmoothing:
    def __init__(self, smoothing_factor: int = 20):

        self.smoothing_factor = smoothing_factor

    def smooth(self, current_value: int, target_value: int) -> int:
        """
        Smooth the PWM signal to the motor controller.
        Used by drive method.
        """

        if (abs(target_value - current_value) > self.smoothing_factor):
            if target_value > current_value:
                current_value += self.smoothing_factor
            else:
                current_value -= self.smoothing_factor
        else:
            current_value = target_value

        return current_value
