import numpy as np
from typing import Tuple, List

class Robot:
    """
    Class that represents the robot for the MPC.

    Attributes:
        h: height of the COM of the robot
        foot_dimensions: (width, length) of the foot
        spacing_x: distance between the feet in the x direction
        spacing_y: distance between the feet in the y direction
        com_position: (x, y) position of the COM
        com_velocity: (x_dot, y_dot) velocity of the COM
        com_acceleration: (x_ddot, y_ddot) acceleration of the COM
        cop_position: (x, y) position of the COP
        ----------------------
        The left and right foot positions are calculated by the MPC
        left_foot_position: (x, y) position of the left foot
        right_foot_position: (x, y) position of the right foot
    """
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
        self.offline_com_trajectory = []
        self.offline_left_foot_trajectory = []
        self.offline_right_foot_trajectory = []

    def initialize_position(self,
                            xk,
                            yk,
                            g) -> None:
        """
        Initializes the position of the robot.
        Args:
            xk: (x, x_dot, x_ddot)
            yk: (y, y_dot, y_ddot)
            g: gravity

        Returns:

        """
        self.com_position = np.array([xk[0], yk[0]])
        self.com_velocity = np.array([xk[1], yk[1]])
        self.com_acceleration = np.array([xk[2], yk[2]])
        self.cop_position = (np.array([1, 0, -self.h / g]) @ xk, np.array([1, 0, -self.h / g]) @ yk)
        self.left_foot_position = np.array([xk[0], yk[0] + self.spacing_y/2 + self.foot_dimensions[1]/2])

    def set_foot_positions_closed_loop(self,
                                       steps: List[Tuple[float, float, str, float]])-> None:
        """
        Sets the position of the left and right foot based on the current step.
        Using the current cop position associated to the robot, the foot position is set to the closest step.
        For the double support phase, the foot positions are calculated using the dimensions and spacing of the feet
        # TODO: Not yet generalized for different trajectory scenarios
        Args:
            steps: (coord_x, coord_y, left/right/double_support)

        Returns:

        """
        foot_position = np.array([steps[0][0], steps[0][1], steps[0][3]])
        foot = steps[0][2]
        if foot == "left":
            self.left_foot_position = foot_position
            self.right_foot_position = None
        elif foot == "right":
            self.right_foot_position = foot_position
            self.left_foot_position = None
        elif foot == "double_support":
            self.right_foot_position = foot_position - np.array([0, self.spacing_y / 2 + self.foot_dimensions[1] / 2, 0])
            self.left_foot_position = foot_position + np.array([0, self.spacing_y / 2 + self.foot_dimensions[1] / 2, 0])
        min_dist = abs((self.cop_position[0] - foot_position[0]) ** 2 + (self.cop_position[1] - foot_position[1]) ** 2)
        for step in steps:
            dist = abs((self.cop_position[0] - step[0]) ** 2 + (self.cop_position[1] - step[1]) ** 2)
            foot = step[2]
            if dist < min_dist:
                min_dist = dist
                if foot == "left":
                    self.left_foot_position = np.array([step[0], step[1], step[3]])
                    self.right_foot_position = None
                elif foot == "right":
                    self.right_foot_position = np.array([step[0], step[1], step[3]])
                    self.left_foot_position = None
                else:
                    foot_position = np.array([step[0], step[1], step[3]])
                    self.right_foot_position = foot_position - np.array(
                        [0, self.spacing_y / 2 + self.foot_dimensions[1] / 2, 0])
                    self.left_foot_position = foot_position + np.array(
                        [0, self.spacing_y / 2 + self.foot_dimensions[1] / 2, 0])

    def set_positional_attributes(self,
                                  xk: Tuple[float, float, float],
                                  yk: Tuple[float, float, float],
                                  steps: List[Tuple[float, float, str, float]],
                                  g: float) -> None:
        """
        Sets the position of the robot based on the current state.
        Args:
            xk: (x, x_dot, x_ddot)
            yk: (y, y_dot, y_ddot)
            steps: List of steps in the current prediction horizon (coord_x, coord_y, left/right/double_support)
            g: gravity

        Returns:

        """
        self.com_position = np.array([xk[0], yk[0]])
        self.com_velocity = np.array([xk[1], yk[1]])
        self.com_acceleration = np.array([xk[2], yk[2]])
        self.cop_position = (np.array([1, 0, -self.h / g]) @ xk, np.array([1, 0, -self.h / g]) @ yk)
        self.set_foot_positions_closed_loop(steps)

        self.offline_com_trajectory.append(self.com_position)
        self.offline_left_foot_trajectory.append(self.left_foot_position)
        self.offline_right_foot_trajectory.append(self.right_foot_position)
        assert(len(self.offline_left_foot_trajectory) == len(self.offline_right_foot_trajectory) ==
               len(self.offline_com_trajectory))

        for i in range(len(self.offline_com_trajectory)):
            if self.offline_com_trajectory[i] is not None:
                self.offline_com_trajectory[i] = tuple(self.offline_com_trajectory[i])
            if self.offline_left_foot_trajectory[i] is not None:
                self.offline_left_foot_trajectory[i] = tuple(self.offline_left_foot_trajectory[i])
            if self.offline_right_foot_trajectory[i] is not None:
                self.offline_right_foot_trajectory[i] = tuple(self.offline_right_foot_trajectory[i])



    def get_positional_attributes(self):
        return self.com_position, self.com_velocity, self.com_acceleration, self.cop_position, \
               self.left_foot_position, self.right_foot_position

