#!/usr/bin/env python3

from simple_pid import PID

class PIDController:
    """
    A wrapper class for the simple_pid library.
    """

    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0):
        """
        Initialize the PID controller with gains and an initial setpoint.

        Parameters:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            setpoint (float): Initial setpoint (default is 0.0).
        """
        self._pid = PID(kp, ki, kd, setpoint=setpoint)
        self._pid.output_limits = (-1, 1)

    def updateSetpoint(self, setpoint: float) -> None:
        """
        Update the setpoint for the PID controller.

        Parameters:
            setpoint (float): The new setpoint.
        """
        self._pid.setpoint = setpoint

    def stabilize(self, measured_value: float) -> float:
        """
        Compute PID output while considering yaw wrap-around.

        Parameters:
            measured_value (float): The current measured value from the IMU.

        Returns:
            float: The computed control output.
        """
        error = self._angleDifference(measured_value, self._pid.setpoint)
        return self._pid(measured_value + error)
    
    def _angleDifference(self, a: float, b: float) -> float:
        """
        Calculate the shortest difference between two angles as the IMU range is in cyclic angles [-180,180].

        Parameters:
            a (float): The first angle [Actual measurement].
            b (float): The second angle [Target measurement].

        Returns:
            float: The smallest difference between the two angles.
        """
        diff = (a - b + 180) % 360 - 180  
        return diff