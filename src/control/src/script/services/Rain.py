#!/usr/bin/env python3
import RPi.GPIO as GPIO
from services.Logger import Logger
from DTOs.LogSeverity import LogSeverity

class Rain:
    def __init__(self, pin):
        self.gpio = GPIO 
        self.gpio.setmode(GPIO.BCM)
        self.gpio.setup(pin, GPIO.IN)
        self.pin = pin

    def checkWater(self):
        status = self.gpio.input(self.pin)
        if status:
            Logger.logToFile(LogSeverity.INFO, "No Water Detected", "Rain Sensor")
            Logger.logToGUI(LogSeverity.INFO, "No Water Detected", "Rain Sensor")
        else:
            Logger.logToFile(LogSeverity.FATAL, "Water Detected!!!", "Rain Sensor")
            Logger.logToGUI(LogSeverity.FATAL, "Water Detected!!!", "Rain Sensor")
        return status