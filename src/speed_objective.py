from visuals import *
from simulations import *

def main():
    # Problem variables
    T_pred = 100e-3  # (s)
    T_control = 50e-3  # (s)
    simulation_time = 10  # (s)
    prediction_time = 2  # (s)
    g = 9.81
    h = 0.8
    xk_init = (0, 0, 0)
    yk_init = (0, 0, 0)
    alpha = 1  # Weight for jerk
    gamma = 1  # Weight for zk_ref
    beta = 1   # Weight for velocity

    # Footstep planning
    foot_dimensions = [0.3, 0.25]  # length(x), width(y)
    spacing = 0.4  # lateral spacing between feet
    duration_double_init = 0.08
    duration_step = 0.08
    steps = int(simulation_time / T_control)

    zk_min_x, zk_max_x = construct_zmin_zmax_moving(steps, duration_double_init, duration_step,
                                                    foot_dimensions[0])
    zk_ref_x = (zk_min_x + zk_max_x)/2

    zk_min_y, zk_max_y = construct_zmin_zmax(steps, duration_double_init, duration_step,
                                             foot_dimensions[1], spacing)
    zk_ref_y = (zk_min_y + zk_max_y)/2

    theta_ref = 0 * np.ones(steps)  # radians

    speed_ref_x = construct_speed_ref(steps, duration_double_init + duration_step, stop_at=0.75,
                                      average_speed=0.3)
    speed_ref_y = np.zeros(steps)

    t = np.arange(0, simulation_time, T_control)

    plt.plot(t, zk_ref_x, label="zk_ref_x")
    plt.plot(t, zk_ref_y, label="zk_ref_y")
    plt.legend()
    plt.show()



    # Running the MPC

    cop_x, com_x, cop_y, com_y = qp_speed(simulation_time, prediction_time, T_pred, T_control,
                                            h, g, alpha, gamma, beta, xk_init, yk_init,
                                            zk_ref_x, zk_ref_y, theta_ref, speed_ref_x,
                                            speed_ref_y, foot_dimensions)

    # Plot the results

    plt.plot(cop_x, label="cop")
    plt.plot(com_x, label="com")
    plt.plot(zk_min_x, linewidth=0.7)
    plt.plot(zk_max_x, linewidth=0.7)
    plt.title("x movement")
    # plt.ylim(0,2)
    plt.legend()
    plt.show()

    plt.plot(cop_y, label="cop")
    plt.plot(com_y, label="com")
    plt.plot(zk_min_y, linewidth=0.7)
    plt.plot(zk_max_y, linewidth=0.7)
    # plt.ylim((-0.8, 0.8))
    plt.title("y movement")
    plt.legend()
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(cop_x, cop_y, label="cop", color="green")
    ax.plot(com_x, com_y, label="com", color="red")
    # Plot footsteps
    plot_foot_steps(ax, zk_ref_x, zk_ref_y, theta_ref, foot_dimensions, spacing)
    # Display the plot
    plt.legend()
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    # ax.set_ylim((-0.03, 0.03))
    plt.title("Trajectory of robot")
    plt.show()


if __name__ == "__main__":
    main()