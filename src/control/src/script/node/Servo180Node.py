#!/usr/bin/env python3

import rospy
from services.Servo180 import Servo180
from services.Joystick import CJoystick
from services.PCADriver import PCA
from mock.PCAMock import PCAMock
from utils.Configurator import Configurator

class Servo180Node:
    def __init__(self) -> None:
        rospy.init_node('servo180_node', anonymous=False)
        self.joystick = CJoystick()
        self.__pins =  Configurator().fetchData(Configurator().PINS)
        self.channel = int(self.__pins['STEREO_SERVO'])
        self.servo = Servo180(self.channel, PCA.getInst(),2500,1000)
        self.__up = 'STEREO_SERVO_UP'
        self.__down = 'STEREO_SERVO_DOWN'

    def run(self):
        try:
            while not rospy.is_shutdown():
                # rospy.loginfo(self.joystick.isPressed(self.__up))
                if self.joystick.isPressed(self.__up):
                    self.servo.setStep(5)
                    self.servo.move()
                    rospy.loginfo("Going Up")
                elif self.joystick.isPressed(self.__down):
                    self.servo.setStep(-5)
                    self.servo.move()
                    rospy.loginfo("Going Down")
                else:
                    rospy.loginfo("Stopping")    
        except Exception as e:
            rospy.logerr(f"Error in Servo180Node: {e}")

if __name__ == "__main__":
    try:
        servo = Servo180Node()
        servo.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
