#!/usr/bin/env python3
from smbus2 import SMBus
# from bmp280 import BMP280

import board

import adafruit_bmp280
from adafruit_extended_bus import ExtendedI2C

class Pressure():
    def __init__(self):
        i2c = ExtendedI2C(3)  # opens /dev/i2c-3 and supports try_lock()
        self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c,address=0x76)
        self.bmp280.sea_level_pressure = 1013.25
    def getPressure(self): 
        # pressure = (self.bmp280.get_pressure()- 1500)/1000 
        return self.bmp280.pressure
    
    def getTemperature(self):
        # temperature = self.bmp280.get_temperature()
        return self.bmp280.temperature