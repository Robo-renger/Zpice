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
        self.left_dc = SinglePWMDCMotor(self.pca, self.__pins['DC_LEFT_PCA_CHANNEL'], self.__pins['DC_LEFT_GPIO'])
        self.right_dc = SinglePWMDCMotor(self.pca, self.__pins['DC_RIGH_PCA_CHANNEL'], self.__pins['DC_RIGHT_PCA_GPIO'])
        self.joystick = CJoystick()
        
    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.joystick.isPressed("DCLEFTGRIPPER_RIGHT"):
                    self.left_dc.driveForward()
                elif self.joystick.isPressed("DCLEFTGRIPPER_LEFT"):
                    self.left_dc.driveBackward()
                else:
                    self.left_dc.stop()

                if self.joystick.isPressed("DCRIGHTGRIPPER_RIGHT"):
                    self.right_dc.driveForward()
                elif self.joystick.isPressed("DCRIGHTGRIPPER_LEFT"):
                    self.right_dc.driveBackward()
                else:
                    self.right_dc.stop()
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
        
    
