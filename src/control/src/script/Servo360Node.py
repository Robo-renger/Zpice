#!/usr/bin/env python3

import rospy
from services.Servo360 import Servo360
from services.Joystick import CJoystick
from services.PCADriver import PCA
from mock.PCAMock import PCAMock
from utils.Configurator import Configurator
class Servo360Node:
    def __init__(self,pca):
        rospy.init_node('servo360_node', anonymous=False)
        self.joystick = CJoystick()
        self.photosphereServoPin = Configurator().fetchData(Configurator.PINS)['PHOTOSPHERE_SERVO']
        self.servo = Servo360(self.photosphereServoPin, pca,2100,0,1000)
        self.servo.setDelay(0.0001)

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.joystick.isPressed("PHOTOSPHERE_SERVO_LEFT"):
                    self.servo.goForward()
                elif self.joystick.isPressed("PHOTOSPHERE_SERVO_RIGHT"):
                    self.servo.goBackwards()
                else:
                    self.servo.Stop()    
        except Exception as e:
            rospy.logerr(f"Error in Servo360Node: {e}")
        finally:
            self.joystick.cleanup()

if __name__ == "__main__":
    try:
        servo = Servo360Node(PCA.getInst())
        servo.run()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting...")
