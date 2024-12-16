from .PCADriver import PCA
from check_simulation import check_simulation_mode



class Thruster:
    
    
    def __init__(self, pca_driver: 'PWMDriver', channel: int, simulation_mode: bool):
        self.pca_driver = pca_driver 
        self.channel = channel
        self.simulation_mode = simulation_mode
        self.min_val = 1200
        self.max_val = 1800
        self.init_val = 1500
          
    def drive(self, microseconds: int):
        if microseconds < self.min_val or microseconds > self.max_val:
            raise ValueError("Microseconds must be between 1200 and 1800.")
        
        if simulation_mode:
            rospy.loginfo(f"Setting thruster {self.channel} to {microseconds} microseconds.")
        else:
            self.pca_driver.PWMWrite(self.channel, microseconds)
        
    
    def stop(self):
        self.pca_driver.set_pwm(self.channel, 1500)