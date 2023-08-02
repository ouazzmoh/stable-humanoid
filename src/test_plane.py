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
alpha = 1e-3  # Weight for safety distance of the plane(Higher weight the more it is minimized)
beta = 1e-6 # Weight for smoothing
epsilon = 1e-3 # Precision of how close is the normal of the plane a unit vector

robot_vertices1, person_vertices1 = [], []
robot_vertices2, person_vertices2 = [], []
for i in range(0, round(simulation_time / T_control)):
    robot_vertices1.append((3 + i *0.3, 0, 0))
    person_vertices1.append((0 + i *0.3, 0, 0))

    robot_vertices2.append((3 + i * 0.3, 0, 5))
    person_vertices2.append((0 + i * 0.3 + 2, 0, 3))

robot_vertices = [robot_vertices1, robot_vertices2]
person_vertices = [person_vertices1, person_vertices2]

robot_arm = RobotArm(robot_vertices)
person = Person(person_vertices)


def main():
    plane_solver = SolvePlane(robot_arm, person, simulation_time, beta, alpha, epsilon)
    planes = []
    for i in range(int(simulation_time / T_control) - 1):
        res = plane_solver.run_iteration(i)
        planes.append(res)


    # Number of frames to skip
    n = len(planes) // 50  # adjust this to control the number of frames

    # x range
    x_range = np.linspace(-10, 10, 100)
    z_range = np.linspace(-10, 10, 100)

    # Loop over the planes and plot each nth one
    for i, plane in enumerate(planes):
        print(f"------------- Iteration {i} ---------")
        print(f"Plane : {planes[i]}")
        fig, ax = plt.subplots()

        ax.set_xlabel('X')
        ax.set_ylabel('Z')

        # Print robot vertices
        print("Robot vertices")
        for r_vertex in robot_vertices:
            print(r_vertex[i])
            ax.plot(r_vertex[i][0], r_vertex[i][2], 'ro')

        print("Person vertices")
        for p_vertex in person_vertices:
            print(p_vertex[i])
            ax.plot(p_vertex[i][0], p_vertex[i][2], 'go')

        normal_vector, constant = plane[0], plane[1]

        # get the x-intercept
        x_intercept = constant / normal_vector[0]
        z_intercept = constant / normal_vector[2]

        # plot the line representing the plane in 2D:
        # Fixing the precision of the plotting -> what to consider a 0 ?
        slope = -normal_vector[0] / normal_vector[2] if abs(normal_vector[2]) > 1e-13 else 0

        ax.plot(np.ones_like(z_range) * x_intercept + slope * x_range, x_range, label=f'Plane {i}')
        plt.xlim((0, 35))
        # Save the plot as an image file.
        # Pad for the converter to generate a good gif
        plt.savefig(f"../plane_fig/frame_{i:03}.png")
        plt.clf()


if __name__ == "__main__":
    main()
    subprocess.run("convert -delay 1 ../plane_fig/frame* ../animate_plane.gif", check=True, shell=True)
