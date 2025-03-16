#!/usr/bin/env python3

from simple_pid import PID

class PIDController:
    """
    A wrapper class for the simple_pid library.
    """
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = None):
        """
        Initialize the PID controller with gains and an initial setpoint.

        Parameters:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            setpoint (float): Initial setpoint (default is 0.0).
        """
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._pid = PID(self.kp, self.ki, self.kd, setpoint=setpoint)
        self._pid.output_limits = (-1, 1)
        self.isHeading = False

    def updateSetpoint(self, setpoint: float) -> None:
        """
        Update the setpoint for the PID controller.

        Parameters:
            setpoint (float): The new setpoint.
        """
        self.setpoint = setpoint
        self._pid.setpoint = setpoint
    
    def updateConstants(self, kp: float, ki: float, kd: float) -> None:
        """
        Update the gains for the PID controller.

        Parameters:
            kp (float): The new Proportional gain.
            ki (float): The new Integral gain.
            kd (float): The new Differential gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._pid.Kp = self.kp
        self._pid.Ki = self.ki
        self._pid.Kd = self.kd

    def updateActiveConstants(self, kp_factor: float, ki_factor: float, kd_factor: float):
        """
        Update the gains for the ACTIVE PID controller.

        Parameters:
            kp_factor (float): The Proportional gain factor.
            ki_factor (float): The Integral gain factor.
            kd_factor (float): The Differential gain factor.
        """
        self._pid.Kp = self.kp * kp_factor
        self._pid.Kp = self.ki * ki_factor
        self._pid.Kp = self.kd * kd_factor

    def stabilize(self, measured_value: float) -> float:
        """
        Compute PID output while considering yaw wrap-around.

        Parameters:
            measured_value (float): The current measured value from the IMU.

        Returns:
            float: The computed control output.
        """
        if self._pid.setpoint is not None and measured_value is not None:
            error = 0
            if self.isHeading:
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