from control.src.services.PWMMotor import PWM_Motors

class Thruster(PWM_Motors):
    #Note: Max value was changed from 1900 to 1823 as this is equivelant to thruster max reverse direction PWM
    def __init__(self, pca, channel, min_value=1100, max_val=1823, init_value=1500):
        super().__init__(pca, channel, min_value, max_val, init_value)
