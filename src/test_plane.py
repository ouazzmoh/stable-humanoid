#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from robot_arm import RobotArm
from person import Person
from solve_plane import SolvePlane


T_pred = 100e-3
T_control = 100e-3
simulation_time = 10
prediction_time = 2
alpha = 1  # Weight for safety distance of the plane
beta = 1  # Weight for smoothing
epsilon = 1e-6  # Precision of how close is the normal of the plane a unit vector

robot_vertices = [(0, 0, 0)] * round(simulation_time / T_control)
person_vertices = [(3, 0, 0)] * round(simulation_time / T_control)

robot_arm = RobotArm(robot_vertices)
person = Person(person_vertices)


def main():
    plane_solver = SolvePlane(robot_arm, person, simulation_time, beta, alpha, epsilon)
    res1 = plane_solver.run_iteration(0)

    # Normal vector components and scalar:
    a, b, c, d = 1, 1, 1, 1
    # example values, replace with your own

    # Create a grid of x and y values
    x = np.linspace(-10, 10, 100)
    y = np.linspace(-10, 10, 100)
    x, y = np.meshgrid(x, y)

    # Solve for z values
    z = (d - a * x - b * y) / c

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, z, color='b', alpha=0.5)

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()


if __name__ == "__main__":
    main()
