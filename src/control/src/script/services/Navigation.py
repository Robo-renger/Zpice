from services.PCADriver import PCA
from services.Vectorizer import Vectorizer
from services.Thruster import Thruster

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

    def drive(self, pwm_value):
        self.last_pwm = pwm_value
        print(f"{self.thrusters.get(self.channel)} set to {pwm_value}")

    def stop(self):
        self.last_pwm = 1500 
        print(f"Thruster on channel {self.channel} stopped (set to {self.last_pwm})")

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
        :param value: The speed percentage for the upward thrust (e.g., 0-100).
        """
        try:
            Navigation._activateThrusters({
                "front": value,
                "back": value,
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
        :param value: The speed percentage for the downward thrust (e.g., 0-100).
        """
        try:
            Navigation._activateThrusters({
                "front": value,
                "back": value,
                "front_right": 1500,
                "front_left": 1500,
                "back_right": 1500,
                "back_left": 1500,
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
            Navigation._activateThrusters(vectorized)

        except ValueError as e:
            print(f"Error: {e}")

    @staticmethod
    def _activateThrusters(thrust_values: dict) -> None:
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

        