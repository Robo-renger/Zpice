from zope.interface import Interface

class iPWMMotor(Interface):
    
    def output_raw(self, value: int) -> None:
        """
        Publish a raw PWM signal to the motor controller without smoothing.
        """
        
    def drive(self, value: int) -> None:
        """
        Publish a PWM signal to the motor controller with smoothing.
        """
        
    def _smoothing(self, value: int) -> None:
        """
        Smooth the PWM signal to the motor controller.
        Used by drive method.
        """
        
    def stop(self) -> None:
        """
        Stop the motor.
        """
        
    def _ensure_bounds(self, value: int) -> int:
        """
        Ensure the value is within the bounds of min_value and max_val.
        """