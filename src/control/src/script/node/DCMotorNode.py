#!/usr/bin/env python3
import rospy
from services.SinglePWMMotor import SinglePWMDCMotor
from services.PCADriver import PCA
from services.Joystick import CJoystick
from utils.Configurator import Configurator

class DCNode:
    def __init__(self) -> None:
        rospy.init_node("dc_motor_node", anonymous=False)
        self.pca = PCA.getInst()
        self.__pins = Configurator().fetchData(Configurator().PINS)
        self.verticalDC = SinglePWMDCMotor(self.pca, self.__pins['DC_VERTICALGRIPPER_PCA_CHANNEL'], self.__pins['DC_VERTICALGRIPPER_GPIO'],0,19500)
        self.frontDC = SinglePWMDCMotor(self.pca, self.__pins['DC_FRONTGRIPPER_PCA_CHANNEL'], self.__pins['DC_FRONTGRIPPER_GPIO'])
        self.verticalDC.setPWM(19500,350)
        self.frontDC.setPWM(19500,350)
        self.joystick = CJoystick()
        
    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.joystick.isPressed("DCFRONTGRIPPER_RIGHT"):
                    self.frontDC.driveForward()
                    print("FRONT BACKWARD")
                    
                elif self.joystick.isPressed("DCFRONTGRIPPER_LEFT"):
                    self.frontDC.driveBackward()
                    print("FRONT BACKWARD")
                else:
                    self.frontDC.stop()

                if self.joystick.isPressed("VERTICALGRIPPER_LEFT_RIGHT") == 1:
                    self.verticalDC.driveForward()
                    print("VERTICALGRIPPER FORWARD")

                if self.joystick.isPressed("VERTICALGRIPPER_LEFT_RIGHT") == 2:
                    self.verticalDC.driveBackward()
                    print("VERTICALGRIPPER BACKWARD")
                else:
                    self.verticalDC.stop()
        except Exception as e:
            rospy.logerr(f"Error in DCNode: {e}")
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    try:
        left_gripper = DCNode()
        left_gripper.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
        
    
