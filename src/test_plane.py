#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import subprocess

from robot_arm import RobotArm
from person import Person
from solve_plane import SolvePlane


T_pred = 100e-3
T_control = 100e-3
simulation_time = 10
prediction_time = 2
alpha = 1  # Weight for safety distance of the plane(Higher weight the more it is minimized)
beta = 1 # Weight for smoothing
epsilon = 1e-9 # Precision of how close is the normal of the plane a unit vector

robot_vertices, person_vertices = [], []
for i in range(0, round(simulation_time / T_control)):
    robot_vertices.append((3 + i *0.3, 0, 3 + i *0.3))
    person_vertices.append((0 + i *0.3, 0, 0 + i *0.3))

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
    n = len(planes) // 50  # adjust this to control the number of frames

    # x range
    x_range = np.linspace(-10, 10, 100)
    z_range = np.linspace(-10, 10, 100)

    # Loop over the planes and plot each nth one
    for i, plane in enumerate(planes):
        fig, ax = plt.subplots()

        ax.set_xlabel('X')
        ax.set_ylabel('Z')

        # Print robot vertices
        print(robot_vertices[i][0], person_vertices[i][0])

        ax.plot(robot_vertices[i][0], robot_vertices[i][2], 'ro')
        ax.plot(person_vertices[i][0], person_vertices[i][2], 'go')

        normal_vector, constant = plane[0], plane[1]

        # get the x-intercept
        x_intercept = constant / normal_vector[0]
        z_intercept = constant / normal_vector[2]

        # plot the line representing the plane in 2D
        slope = -normal_vector[0] / normal_vector[2]
        intercept = constant / normal_vector[2]

        ax.plot(np.ones_like(z_range) * intercept + slope * x_range, x_range, label=f'Plane {i}')
        plt.xlim((0, 35))
        # Save the plot as an image file.
        # Change the path and filename as needed.
        plt.savefig(f"../plane_fig/frame_{i}.png")

        # Clear the current figure for next plot
        plt.clf()


if __name__ == "__main__":
    main()
    subprocess.run("convert -delay 10 ../plane_fig/frame* ../animate_plane.gif", check=True, shell=True)
