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
    xk_init = (0, 0, 0)
    yk_init = (0, 0, 0)

    alpha = 1
    gamma = 1
    theta_ref = np.array([np.pi/4 for _ in range(steps)])
    foot_dimensions = np.array([0.12, 0.06])  # length x width

    support_values_forward = [-0.13, -0.01, 0.01, 0.13]
    support_values_lat = [-0.13, -0.07, 0.07, 0.13]

    zk_ref_x = [0] * int(steps * 0.03)
    zk_ref_y = [0] * int(steps * 0.03)
    for i in range(int(steps * 0.03), int(steps * 0.4)):
        zk_ref_x += [i]
        zk_ref_y += [i]
    const_x, const_y = zk_ref_x[-1], zk_ref_y[-1]
    for i in range(int(steps * 0.4), int(steps * 0.6)):
        zk_ref_x += [const_x]
        zk_ref_y += [const_y]
    for i in range(int(steps * 0.6), steps):
        zk_ref_x += [-i + int(steps * 0.6) + const_x]
        zk_ref_y += [const_y]

    # plt.plot(zk_ref_y)
    # plt.plot(zk_ref_x)
    # plt.show()



    cop_x, com_x, cop_y, com_y, t = simulation_qp_coupled(t_step, steps, g, h_com, xk_init, yk_init,
                                                          zk_ref_x, zk_ref_y,
                                                          alpha, gamma, theta_ref, foot_dimensions)

    # cop_lateral, com_lateral, cop_forward, com_forward, zk_min_lat, zk_max_lat, zk_min_forward, zk_max_forward, x = \
    #     simulation_x_y_decoupled(t_step, steps, g, h_com, xk_init, yk_init)

    plt.plot(t, zk_ref_y[:len(cop_y)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(t, cop_y, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(t, com_y, color="red", label="com", linewidth=1)
    plt.title("Lateral motion of the robot")
    plt.legend(loc="upper right")
    plt.show()

    plt.plot(t, zk_ref_x[:len(cop_x)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(t, cop_x, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(t, com_x, color="red", label="com", linewidth=1)
    plt.title("Forward moving motion of the robot")
    plt.legend(loc="upper right")
    plt.show()

    plt.plot(com_x, com_y, label="com trajectory", color="red")
    plt.plot(cop_x, cop_y, label="cop trajectory", color="green")
    plt.title("Trajectory of cop and com with alternating forward and lateral motion")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()

