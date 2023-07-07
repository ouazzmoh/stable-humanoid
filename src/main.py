#!/usr/bin/env python3

from visuals import *
from simulations import *

from robot import Robot
from perturbation import Perturbation
from footstep_planner import FootstepPlanner
from controller import MPC


T_pred = 100e-3  # (s)
T_control = 100e-3  # (s)
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

    # Initialize state of the robot
    robot.initialize_position(xk_init, yk_init, g)

    # Parameters
    N = int(prediction_time / T_pred)
    T = T_pred  # Used for intermediate visualizations
    n_iterations = int(simulation_time / T_control)

    # Run the MPC
    com_x, com_y, cop_x, cop_y = [], [], [], []
    left_foot, right_foot = [], []

    for i in range(n_iterations):
        curr_com, _, _, curr_cop, curr_left, curr_right = robot.get_positional_attributes()
        com_x.append(curr_com[0])
        com_y.append(curr_com[1])
        cop_x.append(curr_cop[0])
        cop_y.append(curr_cop[1])
        left_foot.append(curr_left)
        right_foot.append(curr_right)
        # Run the MPC iteration and update the robot state
        controller.MPC_iteration(i, N, T)


    left_foot_unique = remove_duplicates(left_foot)
    right_foot_unique = remove_duplicates(right_foot)

    print(left_foot_unique)
    print(right_foot_unique)
    """
    [array([0.   , 0.325]), array([0.175, 0.325]), array([0.875, 0.325]), array([1.575, 0.325]), array([2.275, 0.325]), array([2.975, 0.325])]
    [array([ 0.   , -0.325]), array([ 0.525, -0.325]), array([ 1.225, -0.325]), array([ 1.925, -0.325]), array([ 2.625, -0.325]), array([ 3.325, -0.325])]
    """
    # todo: Start with left



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
    trajectory_type = "forward"
    move(trajectory_type, debug=False)


if __name__ == "__main__":
    main()

