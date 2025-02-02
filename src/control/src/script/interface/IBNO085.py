#!/usr/bin/env python3
from zope.interface import Interface
from digitalio import DigitalInOut

class IBNO085(Interface):
    def __init__(self, address: int, reset_pin: DigitalInOut, debug: bool):
      """Initializes the BNO085 sensor object.
      @param address: The I2C address of the sensor.
      @param reset_pin: The reset pin of the sensor.
      @param debug: The debug flag.
      """

    def enableFeature(self, feature: int) -> None:
        """Enables the specified feature.
        @param feature: The feature to enable.
        """
        

    def getAcceleration(self) -> tuple:
        """Returns the acceleration readings.
        @return The acceleration readings (x, y and z).
        """

    def getGyro(self) -> tuple:
        """Returns the gyroscope readings.
        @return The gyroscope readings (x, y and z).
        """
    def getMagnetometer(self) -> tuple:
        """Returns the magnetometer readings.
        @return The magnetometer readings (x, y and z).
        """

    def getQuaternion(self) -> tuple:
        """Returns the quaternion readings.
        @return The quaternion readings (x, y and z).
        """
    def getEulerAngles(self) -> tuple:
        """Returns the Euler angles.
        @return The Euler angles (roll, pitch and yaw).
        """

    def Calibrate(self):
        """Calibrates the sensor.
        """

    