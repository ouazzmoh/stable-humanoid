from utils import *
from simulations import *
import matplotlib.pyplot as plt





def main():
    T_control = 5e-3  # (s) sample time for MPC recalculation
    T_pred = 5e-3  # (s) sample time for the prediction horizon
    N_pred = 300  # Number of samples in the prediction horizon N*t is the time of the prediction horizon
    T_total = 10  # (s) total time for the simulation
    time_steps = int(T_total / T_control)  # Total number of time steps in the simulation

    g = 9.81
    h_com = 0.8
    xk_init = (0, 0, 0)

    alpha = 1e-6
    gamma = 1
    theta_ref = np.array([0 for _ in range(time_steps)])
    foot_dimensions = np.array([0.12, 0.06])  # length x width

    t = np.arange(0, T_total, T_control)

    zk_min_x, zk_max_x = construct_zmin_zmax(steps=time_steps, duration_double_init=0.08,
                                             duration_step=0.08,
                                             foot_size=foot_dimensions[0])
    zk_ref_x = (zk_min_x + zk_max_x)/2

    plt.plot(t, zk_ref_x, label="zk_ref_x")
    # plt.plot(zk_min_x, zk_min_y)
    plt.legend()
    plt.show()

    cop_x, com_x = simulation_qp_simple(T_control, T_pred, time_steps, N_pred, g, h_com, xk_init,
                                        zk_ref_x, alpha, gamma, theta_ref, foot_dimensions)


    plt.plot(t, zk_min_x[:len(cop_x)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(t, zk_max_x[:len(cop_x)], linestyle="--", linewidth=0.2, color="gray")
    plt.plot(t, cop_x, color="green", label="cop", linewidth=0.7)
    # plt.scatter(x, cop, s=0.5)
    plt.plot(t, com_x, color="red", label="com", linewidth=1)
    plt.title("Forward moving motion of the robot")
    plt.legend(loc="upper right")
    plt.show()


if __name__ == "__main__":
    main()

