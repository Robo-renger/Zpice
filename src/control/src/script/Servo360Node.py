#!/usr/bin/env python3

import rospy
from services.Servo360 import Servo360
from services.Joystick import CJoystick
from services.PCADriver import PCA
from mock.PCAMock import PCAMock

class Servo360Node:
    def __init__(self, channel: int, pca, up_button: str, down_button: str):
        rospy.init_node('servo360_node', anonymous=False)
        self.joystick = CJoystick()
        self.servo = Servo360(channel, pca)
        self.up_button = up_button
        self.down_button = down_button
        self.servo.setDelay(0.001)

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.joystick.isClicked(self.up_button):
                    self.servo.goForward()
                    rospy.loginfo("Going Up")
                elif self.joystick.isClicked(self.down_button):
                    self.servo.goBackwards()
                    rospy.loginfo("Going Down")
                else:
                    self.servo.Stop()
                    rospy.loginfo("Stopping")    
        except Exception as e:
            rospy.logerr(f"Error in Servo360Node: {e}")
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    try:
        servo = Servo360Node(9, PCA.getInst(), "SERVO_UP", "SERVO_DOWN")
        servo.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
