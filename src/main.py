from visuals import *
from simulations import *
import scenarios

from robot import Robot
from footstep_planner import FootstepPlanner
from controller import MPC


T_pred = 100e-3  # (s)
T_control = 100e-3  # (s)
simulation_time = 10  # (s)
prediction_time = 2  # (s)
g = 9.81
h = 0.8
foot_dimensions = [0.3, 0.25]  # length(x), width(y)
spacing = 0.4  # lateral spacing between feet
duration_double_init = 0.8 # (s)
duration_step = 0.8 # (s)
steps = int(simulation_time / T_control)
alpha = 1  # Weight for jerk
gamma = 1  # Weight for zk_ref
beta = 1   # Weight for velocity
average_speed = (0.3, 0)
stop_at = (8, 10)  # (s)

robot = Robot(h, foot_dimensions, spacing_x=0, spacing_y=spacing)
def moving_forward():
    # Problem variables
    xk_init = (0, 0, 0)
    yk_init = (0, 0, 0)
    # Footstep planning
    step_planner = FootstepPlanner(robot, simulation_time, duration_double_init,
                                   duration_step, trajectory_type="forward", average_speed=average_speed,
                                   stop_at=stop_at)

    zk_min_x, zk_max_x, zk_min_y, zk_max_y, theta_ref = step_planner.footsteps_to_array(0, simulation_time, T_control)
    # speed_ref_x, speed_ref_y = step_planner.speed_to_array(0, simulation_time, T_control)
    t = np.arange(0, simulation_time, T_control)
    zk_ref_x = (zk_min_x + zk_max_x)/2
    zk_ref_y = (zk_min_y + zk_max_y)/2
    plt.plot(t, zk_ref_x, label="zk_ref_x")
    plt.plot(t, zk_ref_y, label="zk_ref_y")
    plt.legend()
    plt.show()
    # Running the MPC
    controller = MPC(simulation_time, prediction_time, T_control, T_pred, robot, step_planner,
                     alpha, beta, gamma, xk_init, yk_init, debug=False)


    cop_x, com_x, cop_y, com_y = controller.run_MPC()

    # Plot the results
    plot_results(cop_x, com_x, cop_y, com_y, zk_min_x, zk_max_x, zk_min_y, zk_max_y, theta_ref,
                 foot_dimensions, spacing)



def moving_upwards():
    # Problem variables
    xk_init = (0, 0, 0)
    yk_init = (0, 0, 0)
    # Footstep planning
    step_planner = FootstepPlanner(robot, simulation_time, duration_double_init,
                                   duration_step, trajectory_type="upwards", average_speed=average_speed,
                                   stop_at=stop_at)

    zk_min_x, zk_max_x, zk_min_y, zk_max_y, theta_ref = step_planner.footsteps_to_array(0, simulation_time, T_control)
    # speed_ref_x, speed_ref_y = step_planner.speed_to_array(0, simulation_time, T_control)
    t = np.arange(0, simulation_time, T_control)
    zk_ref_x = (zk_min_x + zk_max_x) / 2
    zk_ref_y = (zk_min_y + zk_max_y) / 2
    plt.plot(t, zk_ref_x, label="zk_ref_x")
    plt.plot(t, zk_ref_y, label="zk_ref_y")
    plt.legend()
    plt.show()
    # Running the MPC
    controller = MPC(simulation_time, prediction_time, T_control, T_pred, robot, step_planner,
                     alpha, beta, gamma, xk_init, yk_init, debug=True)

    cop_x, com_x, cop_y, com_y = controller.run_MPC()

    # Plot the results
    plot_results(cop_x, com_x, cop_y, com_y, zk_min_x, zk_max_x, zk_min_y, zk_max_y, theta_ref,
                 foot_dimensions, spacing)










def main():
    # moving_forward()
    moving_upwards()

if __name__ == "__main__":
    main()