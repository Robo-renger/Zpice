#!/usr/bin/env python3

from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity

class PWMMapper:
    """
    Static class for mapping normalized values to PWM ranges.
    """
    @staticmethod
    def axesToPWM(value, min_pwm=1220, max_pwm=1780):
        """
        Map a normalized value (-1 to 1) to a PWM range (min_pwm to max_pwm).

        :param value: The normalized value (-1 to 1).
        :param min_pwm: The minimum PWM value (default 1100).
        :param max_pwm: The maximum PWM value (default 1900).

        :return: The corresponding PWM value.
        """
        if not -1 <= value <= 1:
            Logger.logToFile(LogSeverity.ERROR, "Value must be between -1 and 1.", "PWMMapper_axesToPWM")
            Logger.logToGUI(LogSeverity.ERROR, "Value must be between -1 and 1.", "PWMMapper_axesToPWM")
            raise ValueError("Value must be between -1 and 1.")
        
        neutral = (min_pwm + max_pwm) // 2
        if value < 0:  # Reverse
            return int(neutral + (value * (neutral - min_pwm)))
        else:  # Forward
            return int(neutral + (value * (max_pwm - neutral)))

    @staticmethod
    def percentageToPWM(percentage, reverse=False, min_pwm=1220, max_pwm=1780):
        """
        Convert a percentage (0-100) to a PWM value within a dynamic range.

        :param percentage: The speed percentage (0-100).
        :param reverse: If True, map to reverse thrust (min_pwm to neutral).
        :param min_pwm: The minimum PWM value (default 1100).
        :param max_pwm: The maximum PWM value (default 1900).
        
        :return: The corresponding PWM value.
        """
        if not 0 <= percentage <= 100:
            Logger.logToFile(LogSeverity.ERROR, "Percentage must be between 0 and 100.", "PWMMapper_percentageToPWM")
            Logger.logToGUI(LogSeverity.ERROR, "Percentage must be between 0 and 100.", "PWMMapper_percentageToPWM")
            raise ValueError("Percentage must be between 0 and 100.")
        
        neutral = (min_pwm + max_pwm) // 2
        if reverse:
            return int(neutral - (percentage / 100) * (neutral - min_pwm))
        else:
            return int(neutral + (percentage / 100) * (max_pwm - neutral))