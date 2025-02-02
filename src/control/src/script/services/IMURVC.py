#!/usr/bin/env python3
import time
from serial import Serial
from adafruit_bno08x_rvc import BNO08x_RVC

class BNO085RVC:
    def __init__(self, path: str, baudrate: int = 115200):
        uart = Serial(path, baudrate)
        self.rvc = BNO08x_RVC(uart)

    def getReadings(self):
        yaw, pitch, roll, x_accel, y_accel, z_accel = self.rvc.heading
        print("Yaw: %2.2f Pitch: %2.2f Roll: %2.2f Degrees" % (yaw, pitch, roll))
        print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))
        print("")
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        imu = BNO085RVC("/dev/serial0", 115200)
        while True:
            imu.getReadings()
    except KeyboardInterrupt:
        print("Stopping...")

    