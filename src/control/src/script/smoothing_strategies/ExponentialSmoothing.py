from zope.interface import implementer

from interface.iSmoothingStrategy import ISmoothingStrategy


@implementer(ISmoothingStrategy)
class ExponentialSmoothing:
    def __init__(self, alpha: float = 0.1):

        if not (0 < alpha < 1):
            raise ValueError("Alpha must be between 0 and 1.")
        self.alpha = alpha

    def smooth(self, current_value: int, target_value: int) -> int:

        smoothed_value = (1 - self.alpha) * current_value + \
            self.alpha * target_value
        return int(smoothed_value)
