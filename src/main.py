#!/usr/bin/env python3

from visuals import *
from simulations import *

from robot import Robot
from perturbation import Perturbation
from footstep_planner import FootstepPlanner
from controller import MPC


T_pred = 100e-3  # (s)
T_control = 5e-3  # (s)
simulation_time = 10  # (s)
prediction_time = 2  # (s)
g = 9.81
h = 0.8
foot_dimensions = [0.3, 0.25]  # length(x), width(y)
spacing = (0.1, 0.4)  # lateral spacing between feet
duration_double_init = 0.8  # (s)
duration_step = 0.8  # (s)
steps = int(simulation_time / T_control)
alpha = 1  # Weight for jerk
gamma = 1e3  # Weight for zk_ref
beta = 1   # Weight for velocity
average_speed = (0.3, 0)
stop_at = (8, 10)  # (s)

robot = Robot(h, foot_dimensions, spacing_x=spacing[0], spacing_y=spacing[1])


def move(trajectory_type, debug=False, store=False, perturbations=None):
    # Problem variables
    xk_init = (0, 0, 0)
    yk_init = (0, 0, 0)
    # Footstep planning
    step_planner = FootstepPlanner(robot, simulation_time, duration_double_init,
                                   duration_step, trajectory_type=trajectory_type, average_speed=average_speed,
                                   stop_at=stop_at)

    zk_min_x, zk_max_x, zk_min_y, zk_max_y, theta_ref = step_planner.footsteps_to_array(0, simulation_time, T_control)

    plt.plot(zk_min_x, label="zk_min_x")
    plt.plot(zk_max_x, label="zk_max_x")
    plt.show()
    plt.plot(zk_min_y, label="zk_min_y")
    plt.plot(zk_max_y, label="zk_max_y")
    plt.show()


    # speed_ref_x, speed_ref_y = step_planner.speed_to_array(0, simulation_time, T_control)
    t = np.arange(0, simulation_time, T_control)
    zk_ref_x = (zk_min_x + zk_max_x)/2
    zk_ref_y = (zk_min_y + zk_max_y)/2
    plt.plot(t, zk_ref_x, label="zk_ref_x")
    plt.plot(t, zk_ref_y, label="zk_ref_y")
    plt.xlabel("time (s)")
    plt.ylabel("cop_reference (m)")
    plt.title("footstep references in time")
    plt.legend()
    plt.show()
    # Running the MPC
    controller = MPC(simulation_time, prediction_time, T_control, T_pred, robot, step_planner,
                     alpha, beta, gamma, xk_init, yk_init, write_hdf5=store, debug=debug, perturbations=perturbations)
    cop_x, com_x, cop_y, com_y = controller.run_MPC()

    # Plot the results
    plt.plot(t, cop_x, label="cop")
    plt.plot(t, com_x, label="com")
    plt.plot(t, zk_min_x, linewidth=0.7)
    plt.plot(t, zk_max_x, linewidth=0.7)
    plt.title("x movement")
    plt.xlabel("time (s)")
    plt.ylabel("x (m)")
    # plt.ylim(0,2)
    plt.legend()
    plt.show()

    plt.plot(t, cop_y, label="cop")
    plt.plot(t, com_y, label="com")
    plt.plot(t, zk_min_y, linewidth=0.7)
    plt.plot(t, zk_max_y, linewidth=0.7)
    # plt.ylim((-0.8, 0.8))
    plt.title("y movement")
    plt.xlabel("time (s)")
    plt.ylabel("x (m)")
    plt.legend()
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(cop_x, cop_y, label="cop", color="green")
    ax.plot(com_x, com_y, label="com", color="red")
    # Plot footsteps
    plot_foot_steps(ax, (zk_min_x + zk_max_x) / 2, (zk_min_y + zk_max_y) / 2, theta_ref, foot_dimensions, spacing[1])
    # Display the plot
    plt.legend()
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    # ax.set_ylim((-0.03, 0.03))
    plt.title("Trajectory of robot")
    plt.show()


def main():
    # trajectory_type = input("Enter trajectory type: ")
    trajectory_type = "forward"
    perturbations = [Perturbation(0, 0.6, 6)]
    move(trajectory_type, debug=True)


if __name__ == "__main__":
    main()

