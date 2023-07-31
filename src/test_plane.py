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

robot_vertices, person_vertices = [], []
for i in range(0, round(simulation_time / T_control)):
    robot_vertices.append((3 + i *0.3, 0, 0))
    person_vertices.append((0 + i *0.3 , 0, 0))

robot_arm = RobotArm(robot_vertices)
person = Person(person_vertices)


def main():
    plane_solver = SolvePlane(robot_arm, person, simulation_time, beta, alpha, epsilon)
    planes = []
    for i in range(99):
        res = plane_solver.run_iteration(i)
        planes.append(res)

    print(planes)
    # Number of frames to skip
    n = len(planes) // 10  # adjust this to control the number of frames

    x = np.linspace(-10, 10, 20)
    y = np.linspace(-10, 10, 20)
    x, y = np.meshgrid(x, y)

    # Loop over the planes and plot each nth one
    for i, plane in enumerate(planes[::n]):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        print(robot_vertices[i][0], person_vertices[i][2])

        plt.plot(robot_vertices[i][0], robot_vertices[i][1], robot_vertices[i][2], 'ro')
        plt.plot(person_vertices[i][0], person_vertices[i][1], person_vertices[i][2], 'go')

        normal_vector, constant = plane[0], plane[1]

        # We know that normal_vector . <x,y,z> = constant, so we can solve for z
        z = (constant - normal_vector[0] * x - normal_vector[1] * y) / normal_vector[2]

        # Plot the plane
        ax.plot_surface(x, y, z, alpha=0.5)

        # Save the plot as an image file.
        # Change the path and filename as needed.
        plt.savefig(f"../plane_fig/frame_{i}.png")

        # Clear the current figure for next plot
        plt.clf()


if __name__ == "__main__":
    main()
