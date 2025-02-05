#!/usr/bin/env python3

# from control.src.services.PWMMotor import PWM_Motors
# from PWMMotor import PWMMotor
from PWM_Motor import PWMMotor

class Thruster(PWMMotor):
    def __init__(self, pca, channel, min_value=1100, max_val=1900, init_value=1500, smoothing_strategy=None):
        super().__init__(pca, channel, min_value, max_val, init_value, smoothing_strategy)
