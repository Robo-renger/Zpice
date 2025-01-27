from zope.interface import implementer
from interface.iPWM_Motors import iPWM_Motors

@implementer(iPWM_Motors)
class PWM_Motors:
    def __init__(self, pca, channel, min_value, max_val, init_value = 1500):
        self.pca = pca
        self.channel = channel
        self.min_value = min_value
        self.max_val = max_val
        self.current_value = init_value
        self.__smoothing = 0
        
        self.stop() # Initialize the motor by 1500 value

    def output_raw(self, value: int) -> None:
        """
        Publish a raw PWM signal to the motor controller without smoothing.
        """
        if value < self.min_value or value > self.max_val:
            raise ValueError(f"Value must be between {self.min_value} and {self.max_val}.")
        self.pca.PWMWrite(self.channel, value)

    def drive(self, value: int, en_smoothing: bool = True) -> None:

        if value < self.min_value or value > self.max_val:
            raise ValueError(f"Value must be between {self.min_value} and {self.max_val}.")
        
        value = self._ensure_bounds(value) 
        
        if en_smoothing:
            self._smoothing(value)
        
        else:
            self.pca.PWMWrite(self.channel, value)
        
        self.__smoothing = self._smoothing(value)
        self.pca.PWMWrite(0, self.__smoothing)

    def _smoothing(self, value: int) -> None:
        """
        Smooth the PWM signal to the motor controller.
        Used by drive method.
        """
        smoothing_factor = 20
        
        if(abs(value - self.current_value) > smoothing_factor):
            if value > self.current_value:
                self.current_value += smoothing_factor
            else:
                self.current_value -= smoothing_factor
            self.current_value = value
            
        if self.pca is not None:
            self.pca.PWMWrite(self.channel, self.current_value)
        else:
            print("PCA is not initialized.")
        
        time.sleep(0.01)
        
    def stop(self) -> None:
        """
        Stop the motor.
        """
        self.pca.PWMWrite(self.channel, 1500)
        
    def _ensure_bounds(self, value: int) -> int:
        """
        Ensure the value is within the bounds of min_value and max_val.
        """
        if value < self.min_value:
            return self.min_value
        elif value > self.max_val:
            return self.max_val
        return value

