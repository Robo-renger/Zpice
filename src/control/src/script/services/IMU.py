#!/usr/bin/env python3
import time
import math
import adafruit_bno08x as bno
from interface.IBNO085 import IBNO085
from zope.interface import implementer
from digitalio import DigitalInOut
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_extended_bus import ExtendedI2C

@implementer(IBNO085)
class BNO085:
    def __init__(self, address: int = 0x4b, reset_pin: DigitalInOut = None, debug: bool =False):
        i2c = ExtendedI2C(3, frequency=400000)
        self.bno = BNO08X_I2C(i2c, address=address, reset=reset_pin, debug=debug)

    def enableFeature(self, feature: int) -> None:
        self.bno.enable_feature(feature)

    def getAcceleration(self) -> tuple:
        accel_x, accel_y, accel_z = self.bno.acceleration  
        return accel_x, accel_y, accel_z

    def getGyro(self) -> tuple:
        gyro_x, gyro_y, gyro_z = self.bno.gyro 
        return gyro_x, gyro_y, gyro_z

    def getMagnetometer(self) -> tuple:
        mag_x, mag_y, mag_z = self.bno.magnetic 
        return mag_x, mag_y, mag_z

    def getQuaternion(self) -> tuple:
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion 
        return quat_i, quat_j, quat_k, quat_real
    
    def getEulerAngles(self) -> tuple:
        q1, q2, q3, q0 = self.bno.quaternion
        yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
        pitch = math.asin(2 * (q0 * q2 - q3 * q1))
        roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        return roll, pitch, yaw
    


    def Calibrate(self) -> None:
        # Needs more testing and improvements
        print("Starting calibration")
        self.bno.begin_calibration()
        print("Calibration started...\n Enabling calibration features...")
        self.enableFeature(bno.BNO_REPORT_MAGNETOMETER)
        self.enableFeature(bno.BNO_REPORT_GAME_ROTATION_VECTOR)
        start_time = time.monotonic()
        calibration_good_at = None
        while True:
            time.sleep(0.1)
            print("Magnetometer:")
            mag_x, mag_y, mag_z = self.bno.magnetic
            print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
            print("")
            print("Game Rotation Vector Quaternion:")
            (game_quat_i, game_quat_j, game_quat_k, game_quat_real) = self.bno.game_quaternion  
            print("I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (game_quat_i, game_quat_j, game_quat_k, game_quat_real))
            print("********************************")
            calibration_status = self.bno.calibration_status
            print("Magnetometer Calibration quality:", bno.REPORT_ACCURACY_STATUS[calibration_status], " (%d)" % calibration_status)
            if not calibration_good_at and calibration_status >= 2:
                calibration_good_at = time.monotonic()
            if calibration_good_at and (time.monotonic() - calibration_good_at > 5.0):
                input_str = input("\n\nEnter S to save or anything else to continue: ")
                if input_str.strip().lower() == "s":
                    self.bno.save_calibration_data()
                    break
                calibration_good_at = None
            print("**************************************************************")
        print("calibration done")
