import matplotlib.pyplot as plt
from utils import *


def D_theta(N):
    D = np.zeros(shape=(2*N, N))
    for j in range(N):
        d_x = np.array([1, -1])
        for k in range(2):
            D[2*j + k, j] = d_x[k]
    return D

def main():
    # Problem variables
    T_pred = 100e-3  # (s)
    T_control = 100e-3  # (s)
    simulation_time = 10  # (s)
    prediction_time = 1  # (s)
    g = 9.81
    h = 0.8
    xk_init = (0, 0, 0)
    alpha = 1e-6  # Weight for jerk
    gamma = 1e-3  # Weight for zk_ref

    # Footstep planning
    foot_size = 0.12
    duration_double_init = 0.08
    duration_step = 0.08
    steps = int(simulation_time / T_control)
    ##
    zk_min, zk_max = construct_zmin_zmax(steps, duration_double_init, duration_step,
                                         foot_size)

    zk_ref = (zk_min + zk_max)/2
    ####
    t = np.arange(0, simulation_time, T_control)
    # plt.plot(t, zk_min, label="zk_min")
    # plt.plot(t, zk_max, label="zk_max")
    plt.plot(t, zk_ref, label="zk_ref")
    plt.legend()
    plt.show()
    ###
    N = int(prediction_time / T_pred)
    Pzu = p_u_matrix(T_pred, h, g, N)
    Pzs = p_x_matrix(T_pred, h, g, N)
    Q = alpha * np.eye(N) + gamma * Pzu.T @ Pzu

    com = []
    com_velocity = []
    com_acceleration = []
    cop = []
    prev_x = xk_init

    # Padding
    zk_ref = np.array(list(zk_ref) + [zk_ref[-1]] * int(prediction_time / T_control))
    for i in range(int(simulation_time / T_control)):
        # Get the current prediction horizon
        zk_ref_pred = zk_ref[i:i + int(prediction_time / T_control)]
        zk_ref_pred = zk_ref_pred[::int(T_pred / T_control)]  # Down sampling
        assert(len(zk_ref_pred) == N)

        zk_min_pred, zk_max_pred = zk_ref_pred - foot_size/2, zk_ref_pred + foot_size/2
        # Solve the optimization problem ove the current prediction horizon
        p = gamma * Pzu.T @ (Pzs @ prev_x - zk_ref_pred)
        # Condition    Zkmin <= Zk <= Zkmax
        G1 = np.vstack((Pzu, -Pzu))
        h_cond1 = np.hstack((zk_max_pred - Pzs @ prev_x, -(zk_min_pred - Pzs @ prev_x)))

        # Simple condition preparing for the general one
        D = D_theta(N)
        b = (foot_size/2) * np.ones(2 * N)
        G2 = D @ Pzu
        h_cond2 = b + D @ (zk_ref_pred - Pzs @ prev_x)

        jerk = solve_qp(P=Q, q=p, G=G2, h=h_cond2, solver="quadprog")
        if jerk is None:
            print(f"Cannot solve the QP at iteration {i}, most likely the value of xk diverges")



        # ################## Debug
        # if i % 100 == 0 or i == 99:
        #     cop_x_s, com_x_s = [], []
        #     cop_y_s, com_y_s = [], []
        #     jerk_x, jerk_y = [], []
        #     for k in range(int(prediction_time/T_pred)):
        #         # Get the next x and y
        #         next_x = next_com(jerk=jerk[k], previous=prev_x, t_step=T_pred)
        #         com_x_s.append(next_x[0])
        #         cop_x_s.append(np.array([1, 0, -h / g]) @ next_x)
        #         # jerk_x.append(jerks[k])
        #     plt.plot(cop_x_s, label="cop_x", color="green")
        #     plt.plot(jerk_x, label="jerk_x", color="red", linewidth="0.8")
        #     plt.plot(zk_ref_pred, label="zk_ref_x", color="green", linewidth="0.4")
        #     plt.title("QP" + str(i + 1))
        #     plt.ylim((-0.6, 0.6))
        #     plt.legend()
        #     plt.show()


        # Apply the first result of the optimization
        next = next_com(jerk=jerk[0], previous=prev_x, t_step=T_pred)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h / g]) @ next)
        # Update the status of the position
        prev_x = next
    plt.plot(cop, label="cop")
    plt.plot(com, label="com")
    plt.plot(zk_min, linewidth=0.7)
    plt.plot(zk_max, linewidth=0.7)
    # plt.ylim((-0.8, 0.8))
    plt.legend()
    plt.show()







if __name__ == "__main__":
    main()



