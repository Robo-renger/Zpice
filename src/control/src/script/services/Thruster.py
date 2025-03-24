#!/usr/bin/env python3
from services.PWM_Motors import PWM_Motors

class Thruster(PWM_Motors):
    def __init__(self, pca, channel, min_value=1220, max_val=1780, init_value=1500):
        self.defaultMin = min_value
        self.defaultMax = max_val
        super().__init__(pca, channel, min_value, max_val, init_value)
        
    def getDefaultMin(self):
        return self.defaultMin
    
    def getDefaultMax(self):
        return self.defaultMax