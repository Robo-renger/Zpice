#!/usr/bin/env python3
import rospy
from services.SinglePWMMotor import SinglePWMDCMotor
from services.PCADriver import PCA
from services.Joystick import CJoystick
from control.msg import Joystick

class DCMotor:

    def __init__(self):
        rospy.init_node("dc_motor_node", anonymous=False)
        self.rightGripperButton_F = False
        self.rightGripperButton_R = False
    def callback(self,data):
        self.rightGripperButton_F = data.button10
        self.rightGripperButton_R = data.button3
    def run(self):
        rospy.Subscriber("/joystick", Joystick, self.callback)
        pca = PCA().getInst()
        rightGripper = SinglePWMDCMotor(pca, 11,8) 
        if self.rightGripperButton_F:
            print("Forward")
            rightGripper.driveForward()
            rightGripper.pcaHabal()
        elif self.rightGripperButton_R:
            print("Reverse")
            rightGripper.driveBackward()
            rightGripper.pcaHabal()
        else:
            print("ana hena")
            rightGripper.stop()
if __name__ == "__main__":
    try:
        node = DCMotor()
        while not rospy.is_shutdown():
            node.run()
    except KeyboardInterrupt:
        print("Exiting...")
