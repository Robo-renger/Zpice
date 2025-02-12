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
from script.LogPublisherNode import LogPublisherNode
from services.PWMFactory import PWMFactory


@implementer(iLoggable)

class Navigation:
    """
    Static class for ROV navigation.
    """
    _thrusters = {
        "front_right": Thruster(pca=PWMFactory().getPWMDriver(), channel = 5),
        "front_left": Thruster(pca=PWMFactory().getPWMDriver(), channel = 1),
        "back_left": Thruster(pca=PWMFactory().getPWMDriver(), channel = 4),
        "back_right": Thruster(pca=PWMFactory().getPWMDriver(), channel = 0),
        "front": Thruster(pca=PWMFactory().getPWMDriver(), channel = 3),
        "back": Thruster(pca=PWMFactory().getPWMDriver(), channel = 2),
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
                "front_left": pwm_value_reverse,
                "back_right": pwm_value_reverse,
                "back_left": pwm_value_reverse,
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
                "front_left": pwm_value_forward,
                "back_right": pwm_value_forward,
                "back_left": pwm_value_forward,
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
                "front_left": pwm_value_reverse,
                "back_right": pwm_value_reverse,
                "back_left": pwm_value_forward,
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
                "front_left": pwm_value_forward,
                "back_right": pwm_value_forward,
                "back_left": pwm_value_reverse,
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
                "front_left": pwm_value_reverse,
                "back_right": pwm_value_forward,
                "back_left": pwm_value_forward,
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
                "front_left": pwm_value_forward,
                "back_right": pwm_value_reverse,
                "back_left": pwm_value_reverse,
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

        