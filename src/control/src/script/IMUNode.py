#!/usr/bin/env python3
import rospy
import board
import time
import adafruit_bno08x as bno
from services.IMU import BNO085
from control.msg import IMU
from digitalio import DigitalInOut

class IMUNode:
    def __init__(self) -> None:
        rospy.init_node("IMUNode")
        self.pub = rospy.Publisher("IMU", IMU, queue_size=10)
        self.msg = IMU()
        self.imu = BNO085()

    def calibrate(self) -> None:
        self.imu.Calibrate()

    def enableFeature(self, feature: int) -> None:
        self.imu.enableFeature(feature)

    def run(self):
        roll, pitch, yaw = self.imu.getEulerAngles()
        self.msg.roll = roll
        self.msg.pitch = pitch 
        self.msg.yaw = yaw
        self.pub.publish()

if __name__ == "__main__":
    try:
        mode = input("Enter mode: c -> Calibration, r -> Read data: ")
        if mode == "c":
            reset_pin = DigitalInOut(board.D5)
            imu = IMUNode(reset_pin, True)
            imu.calibrate()
        else:
            imu = IMUNode()
            imu.enableFeature(bno.BNO_REPORT_ACCELEROMETER)
            imu.enableFeature(bno.BNO_REPORT_GYROSCOPE)
            imu.enableFeature(bno.BNO_REPORT_MAGNETOMETER)
            imu.enableFeature(bno.BNO_REPORT_ROTATION_VECTOR)
            while not rospy.is_shutdown():
                imu.run()

    except KeyboardInterrupt:
        print("Stopping...")


