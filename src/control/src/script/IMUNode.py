#!/usr/bin/env python3
import rospy
import board
import time
import math
import adafruit_bno08x as bno
from services.IMU import BNO085
from control.msg import IMU
from digitalio import DigitalInOut
from exceptions.SensorInitializationError import SensorInitializationError
from exceptions.SensorReadError import SensorReadError
from exceptions.SensorCalibrationError import SensorCalibrationError

class IMUNode:
    def __init__(self) -> None:
        rospy.init_node("IMUNode")
        self.imu = self.initialize_sensor_with_retry()
        self.pub = rospy.Publisher("IMU", IMU, queue_size=5)
        self.msg = IMU()

    def initialize_sensor_with_retry(self) -> BNO085:
        """
        Attempts to initialize the IMU sensor with retries.

        Returns:
            BNO085: The initialized IMU sensor object.
        Raises:
            SensorInitializationError: If the IMU sensor fails to initialize after all retries.
        """
        max_attempts = 5 
        for attempt in range(max_attempts):
            try:
                imu = BNO085()
                return imu
            except SensorInitializationError as e:
                if attempt < max_attempts - 1:
                    time.sleep(1)
                else:
                    raise SensorInitializationError("IMU sensor initialization failed after all retries.")

    def enableFeature(self, feature: int) -> None:
        self.imu.enableFeature(feature)

    def run(self):
        try:
            roll, pitch, yaw = self.imu.getEulerAngles()
            self.msg.roll = math.degrees(roll)
            self.msg.pitch = math.degrees(pitch) 
            self.msg.yaw = math.degrees(yaw)
            self.pub.publish(self.msg)
            # rospy.logerr(self.msg.pitch)
        except SensorReadError as e:
            rospy.logerr(f"{e} Skipping this cycle...")

if __name__ == "__main__":
    try:
        imu = IMUNode()
        imu.enableFeature(bno.BNO_REPORT_ACCELEROMETER)
        imu.enableFeature(bno.BNO_REPORT_GYROSCOPE)
        imu.enableFeature(bno.BNO_REPORT_MAGNETOMETER)
        imu.enableFeature(bno.BNO_REPORT_ROTATION_VECTOR)
        while not rospy.is_shutdown():
            imu.run()
    except SensorInitializationError as e:
        rospy.logerr(f"Sensor initialization failed. {e}")
    except KeyboardInterrupt:
        print("Stopping...")


