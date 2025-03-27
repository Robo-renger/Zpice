#!/usr/bin/env python3
from services.Vectorizer import Vectorizer
from services.Thruster import Thruster
from helpers.PWMMapper import PWMMapper
from DTOs.LogSeverity import LogSeverity
from services.Logger import Logger
from services.PWMFactory import PWMFactory
from utils.Configurator import Configurator
import time
class Navigation:
    """
    Static class for ROV navigation.

    Attributes:
        _thrusters (dict): Dictionary mapping thruster names to Thruster objects.
    """
    # print("ana lgdeed")
    __pins = Configurator().fetchData(Configurator().PINS)

    _thrusters = {
        "front_right": Thruster(pca=PWMFactory().getPWMDriver(), channel = __pins['FRONT_RIGHT_PCA_CHANNEL']),
        "front_left": Thruster(pca=PWMFactory().getPWMDriver(), channel =__pins['FRONT_LEFT_PCA_CHANNEL']),
        "back_left": Thruster(pca=PWMFactory().getPWMDriver(), channel = __pins['BACK_LEFT_PCA_CHANNEL']),
        "back_right": Thruster(pca=PWMFactory().getPWMDriver(), channel = __pins['BACK_RIGHT_PCA_CHANNEL']),
        "front": Thruster(pca=PWMFactory().getPWMDriver(), channel = __pins['FRONT_PCA_CHANNEL']),
        "back": Thruster(pca=PWMFactory().getPWMDriver(), channel = __pins['BACK_PCA_CHANNEL']),
    }
    time.sleep(3)

    @staticmethod
    def setThrusterSpeed(thrusterName: str, min:int , max:int):
        try:
            thruster = Navigation._thrusters[thrusterName]
            thruster.setSpeed(min,max)
        except KeyError:
            print(f"Thruster '{thrusterName}' not found.")
            print("Available thruster names:")
            for name in Navigation._thrusters.keys():
                print(f"- {name}")
    @staticmethod
    def setDefaultSpeed():
        for name, thruster in Navigation._thrusters.items():
            thruster.setSpeed(thruster.getDefaultMin(),thruster.getDefaultMax())
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
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")

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
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")

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
                "back_left": pwm_value_forward,
                "back_right": pwm_value_reverse
            })
        except ValueError as e:
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")

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
                "front_right": pwm_value_reverse,
                "front_left": pwm_value_forward,
                "back_left": pwm_value_forward,
                "back_right": pwm_value_reverse
            })
        except ValueError as e:
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")

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
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")

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
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")

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
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")

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
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")

    @staticmethod
    def navigate(x_axis: float, y_axis: float, z_axis: float, pitch_axis: float, yaw_axis: float) -> None:
        """
        Main method to control ROV navigation based on joystick inputs.

        Parameters:
            x_axis (float): Horizontal joystick input (-1 to 1) (Right is +, Left is -).
            y_axis (float): Horizontal joystick input (-1 to 1) (Forward is +, Backward is -).
            z_axis (float): Vertical joystick input (-1 to 1) (Up is +, Down is -).
            pitch_axis (float): Pitch input (-1 to 1) (Forward tilt is +, Backward tilt is -).
            yaw_axis (float): Rotation input (-1 to 1). (Clockwise is +, Counterclockwise is -).
        """
        try:
            vectorized = Vectorizer.vectorize(Navigation._thrusters,x_axis, y_axis, z_axis, pitch_axis, yaw_axis)
            Navigation._applyThrusts(vectorized)
        except ValueError as e:
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")
            
    @staticmethod
    def _applyThrusts(thrust_values: dict) -> None:
        """
        Activate thrusters based on provided thrust values.
        
        :param thrust_values: Dictionary mapping thruster names to PWM values.
        """
        try:
            if thrust_values is not None:
                for thruster_name, pwm_value in thrust_values.items():
                    if thruster_name in Navigation._thrusters:
                        Navigation._thrusters[thruster_name].drive(pwm_value)
                    else:
                        Logger.logToFile(LogSeverity.ERROR, f"Thruster [{thruster_name}] not found in _thrusters dictionary", "Navigation")
                        Logger.logToGUI(LogSeverity.ERROR, f"Thruster [{thruster_name}] not found in _thrusters dictionary", "Navigation")
        except ValueError as e:
            Logger.logToFile(LogSeverity.ERROR, f"{e}", "Navigation")
            Logger.logToGUI(LogSeverity.ERROR, f"{e}", "Navigation")

    @staticmethod
    def stopAll() -> None:
        """
        Stops all the thrusters.
        """
        for thruster in Navigation._thrusters.values():
            thruster.stop()
        
        