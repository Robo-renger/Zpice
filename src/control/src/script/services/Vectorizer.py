import math

class Vectorizer:
    """
     Static class for ROV Vectorization.
    """
    @staticmethod
    def _mapToPWM(value, min_pwm=1100, max_pwm=1900) -> int:
        """
        Map a normalized value (-1 to 1) to a PWM range (1100 to 1900).
        """
        Vectorizer._validateValue(value)
        neutral_pwm = 1500
        if value < 0:  # Reverse
            return int(neutral_pwm + (value * (neutral_pwm - min_pwm)))
        else:  # Forward
            return int(neutral_pwm + (value * (max_pwm - neutral_pwm)))

    @staticmethod 
    def calculateThrusterSpeeds(x: float, y: float, z: float, pitch: float, yaw: float) -> dict:
        """
        Calculate the PWM signal for each thruster based on joystick x, y, z, pitch, and yaw input.
        Parameters:
            x (float): Horizontal joystick input (-1 to 1) (Right is +, Left is -).
            y (float): Vertical joystick input (-1 to 1) (Forward is +, Backward is -).
            z (float): Vertical joystick input (-1 to 1) (Up is +, Down is -).
            pitch (float): Pitch input (-1 to 1) (Forward tilt is +, Backward tilt is -).
            yaw (float): Rotation input (-1 to 1). (Clockwise is +, Counterclockwise is -).

        Returns:
            dict: PWM signals for each thruster (t1, t2, t3, t4, t5, t6).
        """
        # Thruster angles based on actual placement
        angles = [
                    math.radians(135),  # t1 -> front_right
                    math.radians(45),   # t2 -> front_left
                    math.radians(315),  # t3 -> back_left
                    math.radians(225)   # t4 -> back_right
                ]

        # Initialize thruster speeds
        thruster_speeds = []

        # Handle horizontal thrusters (t1 to t4) with x, y, and yaw inputs
        for angle in angles:
            forward_contrib = y * math.sin(angle)  # Forward/backward contribution
            strafe_contrib = x * math.cos(angle)   # Right/left contribution

            if x == 0 and y == 0:  # No forward/backward or right/left movement
                # Add yaw contribution (opposing pairs for rotation)
                if angle == math.radians(135) or angle == math.radians(315):  # t1 and t3
                    yaw_contrib = -yaw  # Negative yaw contribution for clockwise rotation
                else:  # t2 and t4
                    yaw_contrib = yaw   # Positive yaw contribution for counterclockwise rotation
            else:
                yaw_contrib = 0  # No yaw contribution if moving forward/backward or right/left

            speed = forward_contrib + strafe_contrib + yaw_contrib
            thruster_speeds.append(speed)

        # Handle vertical thrusters (t5 and t6) with z and pitch inputs
        t5_speed =-(z - pitch) # t5 -> front vertical thruster 
        t6_speed =-(z + pitch) # t6 -> back vertical thruster

        thruster_speeds.append(t5_speed)  # t5
        thruster_speeds.append(t6_speed)  # t6

        # Separate horizontal and vertical thruster speeds
        horizontal_speeds = thruster_speeds[:4]  # t1 to t4
        vertical_speeds = thruster_speeds[4:]    # t5 and t6

        # Ensure full thrust utilization for horizontal thrusters
        max_horizontal_input = max(abs(x), abs(y), abs(yaw))  # Maximum horizontal input value
        max_horizontal_speed = max(abs(speed) for speed in horizontal_speeds)  # Maximum horizontal speed

        if max_horizontal_speed > 0:
            horizontal_scaling_factor = max_horizontal_input / max_horizontal_speed  # Scale up to full range
            horizontal_speeds = [speed * horizontal_scaling_factor for speed in horizontal_speeds]

        # Ensure full thrust utilization for vertical thrusters
        max_vertical_input = max(abs(z), abs(pitch))  # Maximum vertical input value
        max_vertical_speed = max(abs(speed) for speed in vertical_speeds)  # Maximum vertical speed

        if max_vertical_speed > 0:
            vertical_scaling_factor = max_vertical_input / max_vertical_speed  # Scale up to full range
            vertical_speeds = [speed * vertical_scaling_factor for speed in vertical_speeds]

        # Combine horizontal and vertical speeds
        thruster_speeds = horizontal_speeds + vertical_speeds

        # Convert normalized speeds to PWM
        pwm_signals = {
            f"t{i+1}": Vectorizer._mapToPWM(speed)
            for i, speed in enumerate(thruster_speeds)
        }

        thrusters = {
            "front_right": pwm_signals["t1"],
            "front_left": pwm_signals["t2"],
            "back_left": pwm_signals["t3"],
            "back_right": pwm_signals["t4"],
            "front": pwm_signals["t5"],
            "back": pwm_signals["t6"]
        }

        return thrusters
    
    @staticmethod
    def _validateValue(value: float) -> None:
        """
        Validates the joystick value.
        :param value: The joystick value to validate.
        :raises ValueError: If the value is out of the allowed range.
        """
        min_value = -1
        max_value = 1
        if value < min_value or value > max_value:
            raise ValueError(f"Value {value} is out of bounds. Must be between {min_value} and {max_value}.")
        