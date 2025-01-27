#!/usr/bin/env python3

from services.check_simulation import check_simulation_mode 
from services.Thruster import Thruster

class MockPCA:
    @staticmethod
    def getInst():
        return MockPCA()

    def PWMWrite(self, channel, microseconds):
        print(f"MockPCA: Setting channel {channel} to {microseconds} microseconds")

    def set_pwm(self, channel, value):
        print(f"MockPCA: Setting channel {channel} to {value}")

class TestThruster:
    def __init__(self):
        self.simulation_mode = check_simulation_mode()
        if self.simulation_mode:
            self.pca_driver = MockPCA.getInst()
        else:
            self.pca_driver = PCA.getInst()
        self.thruster = Thruster(self.pca_driver, channel=0, simulation_mode=self.simulation_mode)

    def test_drive(self, microseconds: int):
        self.thruster.drive(microseconds)
        
    def run_all_tests(self):
        self.test_drive(1200)
        time.sleep(3)
        
        self.test_drive(1800)
        time.sleep(3)
    

if __name__ == "__main__":
    test = TestThruster()
    test.run_all_tests()
    test.thruster.stop()


