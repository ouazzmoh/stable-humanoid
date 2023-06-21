import matplotlib.pyplot as plt
from utils import *


def main():
    # Problem variables
    T_pred = 100e-3  # (s)
    T_control = 5e-3  # (s)
    simulation_time = 10  # (s)
    prediction_time = 1  # (s)
    g = 9.81
    h = 0.8
    xk_init = (0, 0, 0)
    alpha = 0  # Weight for jerk
    gamma = 1  # Weight for zk_ref

    # Footstep planning
    foot_size = 0.12
    duration_double_init = 0.08
    duration_step = 0.08
    steps = int(simulation_time / T_control)
    ##
    zk_min = [-foot_size] * int(duration_double_init * steps)
    zk_max = [foot_size] * int(duration_double_init * steps)
    # Number of steps to take
    periods = int((steps - len(zk_min)) / (duration_step * steps * 2)) - 1
    # First period of steps
    left_min = [-foot_size] * int(steps * duration_step)
    left_max = [0] * int(steps * duration_step)
    right_min = [0] * int(steps * duration_step)
    right_max = [foot_size] * int(steps * duration_step)
    # Multiple periods
    zk_min += (left_min + right_min) * periods
    zk_max += (left_max + right_max) * periods
    zk_min += [-foot_size] * abs((steps - len(zk_min)))
    zk_max += [foot_size] * abs((steps - len(zk_max)))
    zk_min, zk_max = np.array(zk_min), np.array(zk_max)

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
    #  for i in range(1):
        # Get the current prediction horizon
        zk_ref_pred = zk_ref[i:i + int(prediction_time / T_control)]
        zk_ref_pred = zk_ref_pred[::int(T_pred / T_control)]  # Down sampling
        zk_ref_pred = np.array(list(zk_ref_pred) + [zk_ref_pred[-1]] *
                               (int(prediction_time / T_pred) - len(zk_ref_pred)))  # Padding

        assert(len(zk_ref_pred) == N)

        zk_min_pred, zk_max_pred = zk_ref_pred - foot_size/2, zk_ref_pred + foot_size/2
        # Solve the optimization problem ove the current prediction horizon
        p = gamma * Pzu.T @ (Pzs @ prev_x - zk_ref_pred)
        G = np.vstack((Pzu, -Pzu))
        h_cond = np.hstack((zk_max_pred - Pzs @ prev_x, -(zk_min_pred - Pzs @ prev_x)))
        jerk = solve_qp(Q, p, G=G, h=h_cond, solver="quadprog")
        ################## Debug
        if i % 100 == 0:
            cop_x_s, com_x_s = [], []
            cop_y_s, com_y_s = [], []
            jerk_x, jerk_y = [], []
            for k in range(int(prediction_time/T_pred)):
                # Get the next x and y
                next_x = next_com(jerk=jerk[k], previous=prev_x, t_step=T_pred)
                com_x_s.append(next_x[0])
                cop_x_s.append(np.array([1, 0, -h / g]) @ next_x)
                # jerk_x.append(jerks[k])
            plt.plot(cop_x_s, label="cop_x", color="green")
            plt.plot(jerk_x, label="jerk_x", color="red", linewidth="0.8")
            plt.plot(zk_ref_pred, label="zk_ref_x", color="green", linewidth="0.4")
            plt.title("QP" + str(i + 1))
            plt.ylim((-2, 2))
            plt.legend()
            plt.show()
        # Apply the first result of the optimization
        next = next_com(jerk=jerk[0], previous=prev_x, t_step=T_pred)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h / g]) @ next)
        # Update the status of the position
        prev_x = next
    plt.plot(cop, label="cop")
    plt.plot(zk_min, linewidth=0.7)
    plt.plot(zk_max, linewidth=0.7)
    plt.ylim((-0.8, 0.8))
    plt.legend()
    plt.show()







if __name__ == "__main__":
    main()



