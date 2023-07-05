import numpy as np

class Robot:
    def __init__(self,
                 h: float,
                 foot_dimensions: (float, float),
                 spacing_x: float,
                 spacing_y: float) -> None:
        self.h = h
        self.foot_dimensions = foot_dimensions
        self.spacing_x = spacing_x
        self.spacing_y = spacing_y
        self.com_position = None
        self.com_velocity = None
        self.com_acceleration = None
        self.cop_position = None
        self.left_foot_position = None
        self.right_foot_position = None

    def set_positional_attributes(self, xk, yk, g, ref_x=None, ref_y=None):
        self.com_position = np.array([xk[0], yk[0]])
        self.com_velocity = np.array([xk[1], yk[1]])
        self.com_acceleration = np.array([xk[2], yk[2]])
        self.cop_position = (np.array([1, 0, -self.h / g]) @ xk, np.array([1, 0, -self.h / g]) @ yk)
        # TODO : Foot positions
        self.left_foot_position = None
        self.right_foot_position = None


