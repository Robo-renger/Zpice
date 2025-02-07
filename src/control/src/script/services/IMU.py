#!/usr/bin/env python3
import time
import math
import adafruit_bno08x as bno
from interface.IBNO085 import IBNO085
from zope.interface import implementer
from digitalio import DigitalInOut
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_extended_bus import ExtendedI2C
from interface.iLoggable import iLoggable
from exceptions.SensorInitializationError import SensorInitializationError
from exceptions.SensorReadError import SensorReadError
from exceptions.SensorCalibrationError import SensorCalibrationError
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from helpers.JsonFileHandler import JsonFileHandler
from script.LogPublisherNode import LogPublisherNode

@implementer(IBNO085, iLoggable)
class BNO085:
    def __init__(self, address: int = 0x4b, reset_pin: DigitalInOut = None, debug: bool = False):
        """
        Initializes the BNO085 sensor.
        Args:
            address (int): I2C address of the sensor (default is 0x4b).
            reset_pin (DigitalInOut): Reset pin for the sensor.
            debug (bool): Enable debug mode.
        Raises:
            SensorInitializationError: If the sensor fails to initialize.
        """
        self.json_file_handler = JsonFileHandler()
        self.log_publisher = LogPublisherNode()
        try:
            i2c = ExtendedI2C(3, frequency=400000)
            self.bno = BNO08X_I2C(i2c, address=address, reset=reset_pin, debug=debug)
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"BNO085 sensor initialization failed: {str(e)}", "BNO085")
            self.logToGUI(LogSeverity.ERROR, f"BNO085 sensor initialization failed: {str(e)}", "BNO085")
            raise SensorInitializationError(f"BNO085 sensor initialization failed: {str(e)}")

    def enableFeature(self, feature: int) -> None:
        self.bno.enable_feature(feature)

    def getAcceleration(self) -> tuple:
        """        
        Returns:
            tuple: Acceleration in x, y, z axes.
        Raises:
            SensorReadError: If the acceleration data cannot be read.
        """
        try:
            accel_x, accel_y, accel_z = self.bno.acceleration
            return accel_x, accel_y, accel_z
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to read acceleration data: {str(e)}", "BNO085")
            self.logToGUI(LogSeverity.ERROR, f"Failed to read acceleration data: {str(e)}", "BNO085")
            raise SensorReadError(f"Failed to read acceleration data: {str(e)}")

    def getGyro(self) -> tuple:
        """        
        Returns:
            tuple: Gyroscope data in x, y, z axes.
        Raises:
            SensorReadError: If the gyroscope data cannot be read.
        """
        try:
            gyro_x, gyro_y, gyro_z = self.bno.gyro
            return gyro_x, gyro_y, gyro_z
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to read gyroscope data: {str(e)}", "BNO085")
            self.logToGUI(LogSeverity.ERROR, f"Failed to read gyroscope data: {str(e)}", "BNO085")
            raise SensorReadError(f"Failed to read gyroscope data: {str(e)}")
        
    def getMagnetometer(self) -> tuple:
        """
        Returns:
            tuple: Magnetometer data in x, y, z axes.
        Raises:
            SensorReadError: If the magnetometer data cannot be read.
        """
        try:
            mag_x, mag_y, mag_z = self.bno.magnetic
            return mag_x, mag_y, mag_z
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to read magnetometer data: {str(e)}", "BNO085")
            self.logToGUI(LogSeverity.ERROR, f"Failed to read magnetometer data: {str(e)}", "BNO085")
            raise SensorReadError(f"Failed to read magnetometer data: {str(e)}")

    def getQuaternion(self) -> tuple:
        """
        Returns:
            tuple: Quaternion data (i, j, k, real).
        Raises:
            SensorReadError: If the quaternion data cannot be read.
        """
        try:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            return quat_i, quat_j, quat_k, quat_real
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to read quaternion data: {str(e)}", "BNO085")
            self.logToGUI(LogSeverity.ERROR, f"Failed to read quaternion data: {str(e)}", "BNO085")
            raise SensorReadError(f"Failed to read quaternion data: {str(e)}")
    
    def getEulerAngles(self) -> tuple:
        """
        Returns:
            tuple: Euler angles in radians.
        Raises:
            SensorReadError: If the Euler angles cannot be calculated.
        """
        try:
            q1, q2, q3, q0 = self.bno.quaternion
            yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
            pitch = math.asin(2 * (q0 * q2 - q3 * q1))
            roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
            return roll, pitch, yaw
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Failed to calculate Euler angles: {str(e)}", "BNO085")
            self.logToGUI(LogSeverity.ERROR, f"Failed to calculate Euler angles: {str(e)}", "BNO085")
            raise SensorReadError(f"Failed to calculate Euler angles: {str(e)}")
    
    def Calibrate(self) -> None:
        """
        Calibrates the BNO085 sensor.
        
        Raises:
            CalibrationError: If the calibration fails or is aborted.
        """
        try:
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
            print("Calibration done")
        except Exception as e:
            self.logToFile(LogSeverity.ERROR, f"Calibration failed: {str(e)}", "BNO085")
            self.logToGUI(LogSeverity.ERROR, f"Calibration failed: {str(e)}", "BNO085")
            raise SensorCalibrationError(f"Calibration failed: {str(e)}")

    def logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_file_handler.writeToFile(log.toDictionary())
        return log
    
    def logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name)
        return log
