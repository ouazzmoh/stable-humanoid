from utils import *
from simulations import *
import matplotlib.pyplot as plt




def main():
    t_step = 5e-3
    # We simulate from 0 to 9 (s)
    steps = int(9 / t_step)
    g = 9.81
    h_com = 0.8
    r_q = 1e-6
    xk_init = (0.4, 0, 0)
    yk_init = (0, 0, 0)

    alpha = 1
    gamma = 1
    theta_ref = np.array([np.pi/4 for _ in range(steps)])
    foot_dimensions = np.array([0.12, 0.06])  # length x width


    zk_min_x, zk_max_x = construct_zmin_zmax_cos(steps=steps, duration_double_init=0.03,
                                                    duration_step=0.07, duration_transition=0.018, foot_size=0.1,
                                                    spacing=0.14, number_of_periods=3)

    zk_min_y, zk_max_y = construct_zmin_zmax_sin(steps=steps, duration_double_init=0.03,
                                                 duration_step=0.07, duration_transition=0.018, foot_size=0.07,
                                                 spacing=0.14, number_of_periods=3)

    zk_ref_x, zk_ref_y = (zk_min_x + zk_max_x)/2, (zk_min_y + zk_max_y) / 2

    plt.plot(zk_ref_x, label="zk_ref_x", linewidth="1")
    plt.plot(zk_ref_y, label="zk_ref_x", linewidth="1")
    plt.legend()
    plt.show()


    # cop_x, com_x, _, _, zk_min_x, zk_max_x, t = simulation_with_feedback(t_step, steps, g, h_com, r_q,
    #                                                                      xk_init, zk_min_x, zk_max_x)
    #
    # cop_y, com_y, _, _, zk_min_y, zk_max_y, t = simulation_with_feedback(t_step, steps, g, h_com, r_q,
    #                                                                      yk_init, zk_min_y, zk_max_y)
    #
    # plt.plot(t, zk_min_x[:len(cop_x)], linestyle="--", linewidth=0.2, color="gray")
    # plt.plot(t, zk_max_x[:len(cop_x)], linestyle="--", linewidth=0.2, color="gray")
    # plt.plot(t, cop_x, color="green", label="cop", linewidth=0.7)
    # # plt.scatter(x, cop, s=0.5)
    # plt.plot(t, com_x, color="red", label="com", linewidth=1)
    # plt.title("Analytic resolution of the problem")
    # plt.legend(loc="upper right")
    # plt.show()
    #
    # plt.plot(t, zk_min_y[:len(cop_y)], linestyle="--", linewidth=0.2, color="gray")
    # plt.plot(t, zk_max_y[:len(cop_y)], linestyle="--", linewidth=0.2, color="gray")
    # plt.plot(t, cop_y, color="green", label="cop", linewidth=0.7)
    # # plt.scatter(x, cop, s=0.5)
    # plt.plot(t, com_y, color="red", label="com", linewidth=1)
    # plt.title("Analytic resolution of the problem")
    # plt.legend(loc="upper right")
    # plt.show()
    #
    # plt.plot(com_x, com_y, label="com", color="red")
    # plt.plot(cop_x, cop_y, label="cop", color="green")
    # plt.legend()
    # plt.show()

    cop_x, com_x, cop_y, com_y, t = simulation_qp_coupled(t_step, steps, g, h_com, xk_init, yk_init,
                                                          zk_ref_x, zk_ref_y,
                                                          alpha, gamma, theta_ref, foot_dimensions)


    plt.plot(t, zk_ref_y[:len(cop_y)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(t, cop_y, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(t, com_y, color="red", label="com", linewidth=1)
    plt.title("Lateral motion of the robot")
    plt.xlabel("time(s)")
    plt.ylabel("y")
    plt.legend(loc="upper right")
    plt.show()

    plt.plot(t, zk_ref_x[:len(cop_x)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(t, cop_x, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(t, com_x, color="red", label="com", linewidth=1)
    plt.title("Forward motion of the robot")
    plt.xlabel("time(s)")
    plt.ylabel("x")
    plt.legend(loc="upper right")
    plt.show()

    plt.plot(com_x, com_y, label="com trajectory", color="red")
    plt.plot(cop_x, cop_y, label="cop trajectory", color="green")
    plt.title("Trajectory of cop and com")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
