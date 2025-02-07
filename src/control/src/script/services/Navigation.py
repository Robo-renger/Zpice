#!/usr/bin/env python3
from services.PCADriver import PCA
from services.Vectorizer import Vectorizer
from services.Thruster import Thruster
from helpers.PWMMapper import PWMMapper
from interface.iLoggable import iLoggable
from zope.interface import implementer
from helpers.JsonFileHandler import JsonFileHandler
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from nodes.LogPublisherNode import LogPublisherNode

@implementer(iLoggable)
class MockThruster:

    def __init__(self, pca, channel):
        self.pca = pca
        self.channel = channel
        self.last_pwm = None
        self.thrusters = {
            0 : "front",
            1 : "front_right",
            2 : "back_right",
            3 : "back",
            4 : "back_left",
            5 : "front_left",
        }
        self.json_handler = JsonFileHandler()
        self.log_publisher = LogPublisherNode()

    def drive(self, pwm_value):
        self.last_pwm = pwm_value
        self.logToFile(LogSeverity.INFO, f"{self.thrusters.get(self.channel)} set to {pwm_value}", "MockThruster")
        self.logToGUI(LogSeverity.INFO, f"{self.thrusters.get(self.channel)} set to {pwm_value}", "MockThruster")
        print(f"{self.thrusters.get(self.channel)} set to {pwm_value}")
        # self.json_handler.downloadFile("192.168.220.62", "root", "", self.json_handler.file_path, "")

    def stop(self):
        self.last_pwm = 1500 
        print(f"Thruster on channel {self.channel} stopped (set to {self.last_pwm})")

    def logToFile(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.json_handler.writeToFile(log)
        return log

    def logToGUI(self, logSeverity: LogSeverity, msg: str, component_name: str) -> Log:
        log = Log(logSeverity, msg, component_name)
        self.log_publisher.publish(logSeverity.value, msg, component_name)
        return log

class Navigation:
    """
    Static class for ROV navigation.
    """
    # _thrusters = {
    #     "front_right": Thruster(pca=PCA.getInst(simulation_mode = True), channel = 1),
    #     "front_left": Thruster(pca=PCA.getInst(simulation_mode = True), channel = 5),
    #     "back_left": Thruster(pca=PCA.getInst(simulation_mode = True), channel = 4),
    #     "back_right": Thruster(pca=PCA.getInst(simulation_mode = True), channel = 2),
    #     "front": Thruster(pca=PCA.getInst(simulation_mode = True), channel = 0),
    #     "back": Thruster(pca=PCA.getInst(simulation_mode = True), channel = 3),
    # }

    _thrusters = {
        "front_right": MockThruster(pca=None, channel=1),
        "front_left": MockThruster(pca=None, channel=5),
        "back_left": MockThruster(pca=None, channel=4),
        "back_right": MockThruster(pca=None, channel=2),
        "front": MockThruster(pca=None, channel=0),
        "back": MockThruster(pca=None, channel=3),
    }

    @staticmethod
    def moveUp(value) -> None:
        """
        Moves the ROV upward.
        :param value: The speed percentage for the thrust (e.g., 0-100).
        """
        try:
            pwm_value = PWMMapper.percentageToPWM(value, reverse=False)
            Navigation._applyThrusts({
                "front": pwm_value,
                "back": pwm_value,
                "front_right": 1500,
                "front_left": 1500,
                "back_right": 1500,
                "back_left": 1500,
            })
        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def moveDown(value) -> None:
        """
        Moves the ROV downward.
        :param value: The speed percentage for the thrust (e.g., 0-100).
        """
        try:
            pwm_value = PWMMapper.percentageToPWM(value, reverse=True)
            Navigation._applyThrusts({
                "front": pwm_value,
                "back": pwm_value,
                "front_right": 1500,
                "front_left": 1500,
                "back_right": 1500,
                "back_left": 1500,
            })
        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def moveRight(value) -> None:
        """
        Moves the ROV to the right.
        :param value: The speed percentage for the thrust (e.g., 0-100).
        """
        try:
            pwm_value_forward = PWMMapper.percentageToPWM(value, reverse=False)
            pwm_value_reverse = PWMMapper.percentageToPWM(value, reverse=True)
            Navigation._applyThrusts({
                "front": 1500,
                "back": 1500,
                "front_right": pwm_value_reverse,
                "front_left": pwm_value_forward,
                "back_right": pwm_value_reverse,
                "back_left": pwm_value_forward,
            })
        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def moveLeft(value) -> None:
        """
        Moves the ROV to the left.
        :param value: The speed percentage for the thrust (e.g., 0-100).
        """
        try:
            pwm_value_forward = PWMMapper.percentageToPWM(value, reverse=False)
            pwm_value_reverse = PWMMapper.percentageToPWM(value, reverse=True)
            Navigation._applyThrusts({
                "front": 1500,
                "back": 1500,
                "front_right": pwm_value_forward,
                "front_left": pwm_value_reverse,
                "back_right": pwm_value_forward,
                "back_left": pwm_value_reverse,
            })
        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def moveForward(value) -> None:
        """
        Moves the ROV forward.
        :param value: The speed percentage for the thrust (e.g., 0-100).
        """
        try:
            pwm_value_forward = PWMMapper.percentageToPWM(value, reverse=False)
            pwm_value_reverse = PWMMapper.percentageToPWM(value, reverse=True)
            Navigation._applyThrusts({
                "front": 1500,
                "back": 1500,
                "front_right": pwm_value_forward,
                "front_left": pwm_value_forward,
                "back_right": pwm_value_reverse,
                "back_left": pwm_value_reverse,
            })
        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def moveBackward(value) -> None:
        """
        Moves the ROV backward.
        :param value: The speed percentage for the thrust (e.g., 0-100).
        """
        try:
            pwm_value_forward = PWMMapper.percentageToPWM(value, reverse=False)
            pwm_value_reverse = PWMMapper.percentageToPWM(value, reverse=True)
            Navigation._applyThrusts({
                "front": 1500,
                "back": 1500,
                "front_right": pwm_value_reverse,
                "front_left": pwm_value_reverse,
                "back_right": pwm_value_forward,
                "back_left": pwm_value_forward,
            })
        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def rotateClockwise(value) -> None:
        """
        Rotates the ROV clockwise (to the right).
        :param value: The speed percentage for the thrust (e.g., 0-100).
        """
        try:
            pwm_value_forward = PWMMapper.percentageToPWM(value, reverse=False)
            pwm_value_reverse = PWMMapper.percentageToPWM(value, reverse=True)
            Navigation._applyThrusts({
                "front": 1500,
                "back": 1500,
                "front_right": pwm_value_reverse,
                "front_left": pwm_value_forward,
                "back_right": pwm_value_forward,
                "back_left": pwm_value_reverse,
            })
        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def rotateAnticlockwise(value) -> None:
        """
        Rotates the ROV anticlockwise (to the left).
        :param value: The speed percentage for the thrust (e.g., 0-100).
        """
        try:
            pwm_value_forward = PWMMapper.percentageToPWM(value, reverse=False)
            pwm_value_reverse = PWMMapper.percentageToPWM(value, reverse=True)
            Navigation._applyThrusts({
                "front": 1500,
                "back": 1500,
                "front_right": pwm_value_forward,
                "front_left": pwm_value_reverse,
                "back_right": pwm_value_reverse,
                "back_left": pwm_value_forward,
            })
        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def navigate(x_axis: float, y_axis: float, z_axis: float, pitch_axis: float, yaw_axis: float) -> None:
        """
        Main method to control ROV navigation based on joystick inputs.
        """
        try:
            vectorized = Vectorizer.calculateThrusterSpeeds(x_axis, y_axis, z_axis, pitch_axis, yaw_axis)
            Navigation._applyThrusts(vectorized)
        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def _applyThrusts(thrust_values: dict) -> None:
        """
        Activate thrusters based on provided thrust values.
        :param thrust_values: Dictionary mapping thruster names to PWM values.
        """
        try:
            for thruster_name, pwm_value in thrust_values.items():
                if thruster_name in Navigation._thrusters:
                    Navigation._thrusters[thruster_name].drive(pwm_value)
                else:
                    print(f"Thruster '{thruster_name}' not found in _thrusters dictionary.")
        except ValueError as e:
            print(f"Error activating thrusters: {e}")

    @staticmethod
    def stopAll() -> None:
        """
        Stops all thrusters.
        """
        for thruster in Navigation._thrusters.values():
            thruster.stop()

        