#!/usr/bin/env python3
import time
import board
import math
from digitalio import DigitalInOut
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_ROTATION_VECTOR, REPORT_ACCURACY_STATUS
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_extended_bus import ExtendedI2C

class BNO085:
    def __init__(self, reset_pin: DigitalInOut = None, debug: bool =False):
        i2c = ExtendedI2C(3, frequency=400000)
        self.reset_pin = reset_pin
        self.bno = BNO08X_I2C(i2c, address=0x4b, reset=self.reset_pin, debug=debug)

    def enableFeature(self, feature):
        self.bno.enable_feature(feature)

    def Calibrate(self):
        print("Starting calibration")
        self.bno.begin_calibration()
        print("Calibration started...\n Enabling calibration features...")
        self.enableFeature(BNO_REPORT_MAGNETOMETER)
        self.enableFeature(BNO_REPORT_ROTATION_VECTOR)
        start_time = time.monotonic()
        calibration_good_at = None
        while True:
            time.sleep(0.1)
            self.getMagnetometer()
            self.getQuaternion()
            print("********************************")
            calibration_status = self.bno.calibration_status
            print(
                "Magnetometer Calibration quality:",
                REPORT_ACCURACY_STATUS[calibration_status],
                " (%d)" % calibration_status,
            )
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

    def getAcceleration(self):
        print("Acceleration:")
        accel_x, accel_y, accel_z = self.bno.acceleration  
        print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
        print("")
        return accel_x, accel_y, accel_z

    def getGyro(self):
        print("Gyro:")
        gyro_x, gyro_y, gyro_z = self.bno.gyro 
        print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
        print("")
        return gyro_x, gyro_y, gyro_z

    def getMagnetometer(self):
        print("Magnetometer:")
        mag_x, mag_y, mag_z = self.bno.magnetic 
        print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
        print("")
        return mag_x, mag_y, mag_z

    def getQuaternion(self):
        print("Rotation Vector Quaternion:")
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion 
        print(
            "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
        )
        print("")
        return quat_i, quat_j, quat_k, quat_real
    
    def getYaw(self):
        # print("Yaw:")
        q1, q2, q3, q0 = self.bno.quaternion
        yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
        print("Yaw: %0.6f" % math.degrees(yaw))
        return yaw

    def getPitch(self):
        # print("Pitch:")
        q1, q2, q3, q0 = self.bno.quaternion
        pitch = math.asin(2 * (q0 * q2 - q3 * q1))
        print("Pitch: %0.6f" % math.degrees(pitch))
        return pitch


    

if __name__ == "__main__":
    try:
        mode = input("Enter mode: c -> Calibration, r -> Read data: ")
        if mode == "c":
            reset_pin = DigitalInOut(board.D5)
            imu = BNO085(reset_pin, True)
            imu.Calibrate()
        else:
            imu = BNO085()
            imu.enableFeature(BNO_REPORT_ACCELEROMETER)
            imu.enableFeature(BNO_REPORT_GYROSCOPE)
            imu.enableFeature(BNO_REPORT_MAGNETOMETER)
            imu.enableFeature(BNO_REPORT_ROTATION_VECTOR)
            while True:
                time.sleep(0.5)
                # imu.getAcceleration()
                # imu.getGyro()
                # imu.getMagnetometer()
                # imu.getQuaternion()
                imu.getYaw()
                imu.getPitch()
    except KeyboardInterrupt:
        print("Stopping...")