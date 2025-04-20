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
        
        self.stereoServo = Servo180(self.__pins['STEREO_SERVO'], PCA.getInst(),2500,500)
        self.photosphereServo = Servo180(self.__pins['PHOTOSPHERE_SERVO'], PCA.getInst(),2500,500)

    def run(self):
        try:
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                # rospy.loginfo(self.joystick.isPressed(self.__up))
                if self.joystick.isPressed("STEREO_SERVO_UP"):
                    self.stereoServo.setStep(20)
                    self.stereoServo.move()
                    rospy.loginfo("STEREO UP")
                elif self.joystick.isPressed("STEREO_SERVO_DOWN"):
                    self.stereoServo.setStep(-20)
                    self.stereoServo.move()
                    rospy.loginfo("STEREO DOWN")
                    
                if self.joystick.isPressed("PHOTOSPHERE_SERVO_LEFT"):
                    self.photosphereServo.setStep(20)
                    self.photosphereServo.move()
                    rospy.loginfo("PHOTOSPHERE LEFT")
                elif self.joystick.isPressed("PHOTOSPHERE_SERVO_RIGHT"):
                    self.photosphereServo.setStep(-20)
                    self.photosphereServo.move()
                    rospy.loginfo("PHOTOSPHERE RIGHT")
                rate.sleep()  # <-- This is the key fix

        except Exception as e:
            rospy.logerr(f"Error in Servo180Node: {e}")

if __name__ == "__main__":
    try:
        servo = Servo180Node()
        servo.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
