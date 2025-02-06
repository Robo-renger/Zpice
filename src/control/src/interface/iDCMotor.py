from zope.interface import Interface

class iDCMotor(Interface):
    def drive(self) -> None:
        """
        Set the speed of the motor using a PWM signal.
        Speed is given as an 8-bit value (0-255).
        """

    def driveForward(self) -> None:
        """
        Set the speed of the motor using a PWM signal.
        Speed is given as an 8-bit value (0-255).
        """
    
    def driveBackward(self) -> None:
        """
        Set the speed of the motor using a PWM signal.
        Speed is given as an 8-bit value (0-255).
        """
    
    def stop(self) -> None:
        """
        Stop the motor.
        """

    def direction(self, direction: str) -> None:
        """
        Set the direction of the motor.
        Expected values: "f" for forward or "r" for reverse.
        """