#!/usr/bin/env python3

import rospy
from services.Servo180 import Servo180
from services.Joystick import CJoystick
from services.PCADriver import PCA
from mock.PCAMock import PCAMock

class Servo180Node:
    def __init__(self, channel: int, pca, up_button: str, down_button: str):
        rospy.init_node('servo180_node', anonymous=False)
        self.joystick = CJoystick()
        self.servo = Servo180(channel, pca)
        self.up_button = up_button
        self.down_button = down_button

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.joystick.isClicked(self.up_button):
                    self.servo.setStep(5)
                    self.servo.move()
                    rospy.loginfo("Going Up")
                elif self.joystick.isClicked(self.down_button):
                    self.servo.setStep(-5)
                    self.servo.move()
                    rospy.loginfo("Going Down")
                else:
                    rospy.loginfo("Stopping")    
        except Exception as e:
            rospy.logerr(f"Error in Servo180Node: {e}")
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    try:
        servo = Servo180Node(9, PCAMock.getInst(), "SERVO_UP", "SERVO_DOWN")
        servo.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
