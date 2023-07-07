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

    def initialize_position(self, xk, yk, g):
        self.com_position = np.array([xk[0], yk[0]])
        self.com_velocity = np.array([xk[1], yk[1]])
        self.com_acceleration = np.array([xk[2], yk[2]])
        self.cop_position = (np.array([1, 0, -self.h / g]) @ xk, np.array([1, 0, -self.h / g]) @ yk)

    def set_positional_attributes(self, xk, yk, steps, g):
        self.com_position = np.array([xk[0], yk[0]])
        self.com_velocity = np.array([xk[1], yk[1]])
        self.com_acceleration = np.array([xk[2], yk[2]])
        self.cop_position = (np.array([1, 0, -self.h / g]) @ xk, np.array([1, 0, -self.h / g]) @ yk)
        # TODO : Foot positions
        foot_position = np.array([steps[0][0], steps[0][1]])
        # Todo : Make this work with the attribute which_foot stored in the step class
        foot = steps[0][2]
        if foot == "left":
            self.left_foot_position = foot_position
            self.right_foot_position = None
        elif foot == "right":
            self.right_foot_position = foot_position
            self.left_foot_position = None
        elif foot == "double_support":
            # TODO : during double support we give the position of the com for now !
            self.right_foot_position = foot_position - np.array([0, self.spacing_y/2 + self.foot_dimensions[1]/2])
            self.left_foot_position = foot_position + np.array([0, self.spacing_y/2 + self.foot_dimensions[1]/2])
        min_dist = abs((self.cop_position[0] - foot_position[0])**2 + (self.cop_position[1] - foot_position[1])**2)
        for step in steps:
            dist = abs((self.cop_position[0] - step[0])**2 + (self.cop_position[1] - step[1])**2)
            foot = step[2]
            if dist < min_dist:
                min_dist = dist
                if foot == "left":
                    self.left_foot_position = np.array([step[0], step[1]])
                    self.right_foot_position = None
                elif foot == "right":
                    self.right_foot_position = np.array([step[0], step[1]])
                    self.left_foot_position = None
                else :
                    self.right_foot_position = self.com_position
                    self.left_foot_position = self.com_position




