class Perturbation:
    """
    Class that represents a perturbation for the robot
    """
    def __init__(self,
                 value_x: float,
                 value_y: float,
                 time : float):
        self.value_x = value_x  # Perturbation force in x
        self.value_y = value_y  # Perturbation force in y
        self.time = time    # Instant of the perturbation


