class SensorInitializationError(Exception):
    """Raised when the sensor fails to initialize"""
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)