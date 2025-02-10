#!/usr/bin/env python3
import time
from serial import Serial
from adafruit_bno08x_rvc import BNO08x_RVC, RVCReadTimeoutError
from exceptions.SensorReadError import SensorReadError
from exceptions.SensorInitializationError import SensorInitializationError
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity

class BNO085RVC:
    def __init__(self, path: str, baudrate: int = 115200):
        """
        Args:
            path (str): The path to the UART device (e.g., "/dev/serial0").
            baudrate (int): The baud rate for the UART connection (default is 115200).
        Raises:
            SensorInitializationError: If the UART connection fails to initialize.
        """
        try:
            self.uart = Serial(path, baudrate)
            self.rvc = BNO08x_RVC(self.uart)
        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Failed to initialize BNO085RVC sensor: {str(e)}", "BNO085RVC")
            Logger.logToGUI(LogSeverity.ERROR, f"Failed to initialize BNO085RVC sensor: {str(e)}", "BNO085RVC")
            raise SensorInitializationError(f"Failed to initialize BNO085RVC sensor: {str(e)}")

    def getReadings(self) -> tuple:
        """
        Returns:
            tuple: A tuple containing yaw, pitch, roll, x_accel, y_accel, z_accel.        
        Raises:
            SensorReadError: If the sensor fails to read data.
        """
        try:
            yaw, pitch, roll, x_accel, y_accel, z_accel = self.rvc.heading
            return yaw, pitch, roll, x_accel, y_accel, z_accel
        except RVCReadTimeoutError as e:
            Logger.logToFile(LogSeverity.ERROR, f"Failed to read sensor data: {str(e)}", "BNO085RVC")
            Logger.logToGUI(LogSeverity.ERROR, f"Failed to read sensor data: {str(e)}", "BNO085RVC")
            raise SensorReadError(f"Failed to read sensor data: {str(e)}")
        except Exception as e:
            Logger.logToFile(LogSeverity.ERROR, f"Unexpected error while reading sensor data: {str(e)}", "BNO085RVC")
            Logger.logToGUI(LogSeverity.ERROR, f"Unexpected error while reading sensor data: {str(e)}", "BNO085RVC")
            raise SensorReadError(f"Unexpected error while reading sensor data: {str(e)}")

    def close(self) -> None:
        """
        Closes the UART connection.
        """
        if self.uart:
            self.uart.close()
            
if __name__ == "__main__":
    try:
        imu = BNO085RVC("/dev/serial0", 115200)
        while True:
            try:
                yaw, pitch, roll, x_accel, y_accel, z_accel = imu.getReadings()
                print("Yaw: %2.2f Pitch: %2.2f Roll: %2.2f Degrees" % (yaw, pitch, roll))
                print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))
                print("")
                time.sleep(0.1)
            except SensorReadError as e:
                print(e)
                break
    except SensorInitializationError as e:
        print(e)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        imu.close()