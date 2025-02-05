#!/usr/bin/env python3

from Thruster import Thruster
from PCADriver import PCA

from smoothing_strategies.DefaultSmoothing import DefaultSmoothing
from smoothing_strategies.ExponentialSmoothing import ExponentialSmoothing


import time

class TestThruster:
    def __init__(self, pca, channel, smoothing_strategy=None):
        self.thruster = Thruster(pca, channel, smoothing_strategy=smoothing_strategy)

    def test_drive(self, value, en_smoothing=True):

        try:
            while self.thruster.current_value != value:
                self.thruster.drive(value, en_smoothing)
            print(f"Successfully drove thruster to {value}")
            time.sleep(2) 
            self.test_stop()
            
        except ValueError as e:
            print(f"Error: {e}")
            
    def run_all_tests(self):
        
        self.test_drive(1700)
        self.test_drive(1300)
        

    def test_stop(self):
        """
        Test the stop method of the Thruster class.
        """
        print(f"Stopping thruster on channel {self.thruster.channel}")
        self.thruster.stop()
        print("Thruster stopped")

# Example usage:
if __name__ == "__main__":
    
    pca = PCA()  
    channel = 0  
    smoothing_strategy1 = DefaultSmoothing()
    smoothing_strategy2 = ExponentialSmoothing()

    # test_thruster = TestThruster(pca, channel, smoothing_strategy1)
    
    test_thruster = TestThruster(pca, channel, smoothing_strategy2)
    
    # test_thruster.test_drive(1600)  
    
    test_thruster.run_all_tests()

