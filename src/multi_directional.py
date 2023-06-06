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

    support_values_forward = [-0.13, -0.01, 0.01, 0.13]
    support_values_lat = [-0.13, -0.07, 0.07, 0.13]
    zk_min_lat, zk_max_lat = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
                                         duration_right=0.07, duration_transition=0.018,
                                         min_val_left=support_values_lat[0], max_val_left=support_values_lat[1],
                                         min_val_right=support_values_lat[2], max_val_right=support_values_lat[3])
    zk_min_forward, zk_max_forward = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
                                                 duration_right=0.07, duration_transition=0.018,
                                                 min_val_left=support_values_forward[0], max_val_left=support_values_forward[1],
                                                 min_val_right=support_values_forward[2],
                                                 max_val_right=support_values_forward[3])

    cop_lateral, com_lateral, _, _, zk_min_lat, zk_max_lat, x = simulation_qp(t_step, steps, g, h_com, xk_init,
                                                                      zk_min_lat, zk_max_lat)
    cop_forward, com_forward, _, _, zk_min_forward, zk_max_forward, x = simulation_qp(t_step, steps, g, h_com, xk_init,
                                                                                      zk_min_forward, zk_max_forward)

    # cop_lateral, com_lateral, cop_forward, com_forward, zk_min_lat, zk_max_lat, zk_min_forward, zk_max_forward, x = \
    #     simulation_x_y_decoupled(t_step, steps, g, h_com, xk_init, yk_init)

    plt.plot(x, zk_min_lat[:len(cop_lateral)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x, zk_max_lat[:len(cop_lateral)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x, cop_lateral, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(x, com_lateral, color="red", label="com", linewidth=1)
    plt.title("Lateral motion of the robot")
    plt.legend(loc="upper right")
    plt.show()

    plt.plot(x, zk_min_forward[:len(cop_forward)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x, zk_max_forward[:len(cop_forward)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(x, cop_forward, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(x, com_forward, color="red", label="com", linewidth=1)
    plt.title("Forward motion of the robot")
    plt.legend(loc="upper right")
    plt.show()

    plt.plot(com_lateral, com_forward, label="com trajectory", color="red")
    plt.plot(cop_lateral, cop_forward, label="cop trajectory", color="green")
    plt.title("Trajectory of cop and com with alternating forward and lateral motion")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()

