#!/usr/bin/env python3
import rospy
from services.SinglePWMMotor import SinglePWMDCMotor
from services.PCADriver import PCA
from services.Joystick import CJoystick
from control.msg import Joystick
from mock.PCAMock import PCAMock

class DCNode:
    def __init__(self, pca, channel: int, gpio: int, forward_button: str, backward_button: str) -> None:
        rospy.init_node("dc_motor_node", anonymous=False)
        self.pca = pca
        self.dc = SinglePWMDCMotor(pca, channel, gpio)
        self.joystick = CJoystick()
        self.forward_button = forward_button
        self.backward_button = backward_button
        self.channels = [10, 11, 12, 13, 14, 15]
        
    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.joystick.isClicked(self.forward_button):
                    for channel in self.channels:
                        self.pca.PWMWrite(channel, 350)
                    self.dc.driveForward()
                    rospy.loginfo(f"Going Forward: {self.forward_button}")
                elif self.joystick.isClicked(self.backward_button):
                    for channel in self.channels:
                        self.pca.PWMWrite(channel, 19500)
                    self.dc.driveBackward()
                    rospy.loginfo(f"Going Backwards: {self.backward_button}")
                else:
                    self.dc.stop()
                    rospy.loginfo("Stopping")
        except Exception as e:
            rospy.logerr(f"Error in DCNode: {e}")
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    try:
        left_gripper = DCNode(PCA.getInst(), 11, 8, "DCLEFTGRIPPER_LEFT", "DCLEFTGRIPPER_RIGHT")
        left_gripper.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
        
    

# class DCMotor:

#     def __init__(self):
#         rospy.init_node("dc_motor_node", anonymous=False)
#         self.rightGripperButton_F = False
#         self.rightGripperButton_R = False
#     def callback(self,data):
#         self.rightGripperButton_F = data.button10
#         self.rightGripperButton_R = data.button3
#     def run(self):
#         rospy.Subscriber("/joystick", Joystick, self.callback)
#         pca = PCA().getInst()
#         rightGripper = SinglePWMDCMotor(pca, 11,8) 
#         if self.rightGripperButton_F:
#             print("Forward")
#             rightGripper.driveForward()
#             rightGripper.pcaHabal()
#         elif self.rightGripperButton_R:
#             print("Reverse")
#             rightGripper.driveBackward()
#             rightGripper.pcaHabal()
#         else:
#             print("ana hena")
#             rightGripper.stop()
# if __name__ == "__main__":
#     try:
#         node = DCMotor()
#         while not rospy.is_shutdown():
#             node.run()
#     except KeyboardInterrupt:
#         print("Exiting...")
