from services.PWM_Motors import PWM_Motors

class Thruster(PWM_Motors):
    def __init__(self, pca, channel, min_value=1100, max_val=1900, init_value=1500):
        super().__init__(pca, channel, min_value, max_val, init_value)
