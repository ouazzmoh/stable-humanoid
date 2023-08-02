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
beta = 1e-12 # Weight for smoothing
epsilon = 1e-3 # Precision of how close is the normal of the plane a unit vector

robot_vertices1, person_vertices1 = [], []
robot_vertices2, person_vertices2 = [], []
robot_vertices3, person_vertices3 = [], []
robot_vertices4, person_vertices4 = [], []
person_vertices5 = []
person_vertices6 = []

for i in range(0, round(simulation_time / T_control)):
    robot_vertices1.append((8 + i *0.3, 0, 0))
    robot_vertices2.append((5 + i * 0.3, 0, 0))
    robot_vertices3.append((4 + i * 0.3, 0, 5))  # end effector

    person_vertices1.append((1 + i *0.3, 0, 0)) # COM
    person_vertices2.append((1 + i * 0.3, 0, 2)) # HEAD
    person_vertices3.append((0.3 + i * 0.3, 0, -2)) # Left foot
    person_vertices4.append((1.7 + i * 0.3, 0, -2)) # Right foot
    person_vertices5.append((0 + i * 0.3, 0, .5)) # Left hand
    person_vertices6.append((2 + i * 0.3 , 0, .5))  # Right hand

robot_vertices = [robot_vertices1, robot_vertices2, robot_vertices3]
person_vertices = [person_vertices1, person_vertices2, person_vertices3, person_vertices4, person_vertices5,
                   person_vertices6]

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
    x_range = np.linspace(0, 10, 100)
    z_range = np.linspace(0, 10, 100)

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

        x = np.linspace(0, 40 , 100)
        y = slope * x + z_intercept

        ax.plot(x, y, label="Separating Plane")
        plt.xlim((0, 35))
        plt.ylim((-10, 10))
        # Save the plot as an image file.
        # Pad for the converter to generate a good gif
        plt.title("Separating plane between robot(red) and person (green)")
        plt.legend()
        plt.savefig(f"../plane_fig/frame_{i:03}.png")
        plt.clf()


if __name__ == "__main__":
    main()
    subprocess.run("convert -delay 1 ../plane_fig/frame* ../animate_plane.gif", check=True, shell=True)
