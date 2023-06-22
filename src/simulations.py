from utils import *
import matplotlib.pyplot as plt


def simulation_with_feedback(t_step, steps, g, h_com, r_q, xk_init, zk_min, zk_max):
    # zk_min, zk_max = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
    #                                      duration_right=0.07, duration_transition=0.018,
    #                                      min_val_left=support_values[0], max_val_left=support_values[1],
    #                                      min_val_right=support_values[2], max_val_right=support_values[3])

    window_steps = 300
    zk_min = np.array(list(zk_min) + [zk_min[-1]]*window_steps)
    zk_max = np.array(list(zk_max) + [zk_max[-1]]*window_steps)
    zk_ref = (zk_min + zk_max)/2
    com = []
    com_velocity = []
    com_acceleration = []
    cop = []
    prev = xk_init
    possible_trajectories = []
    for i in range(steps):
        jerk = optimal_jerk(t_step=t_step, h_com=h_com, g=g, n=window_steps, xk_init=prev,
                            zk_ref=zk_ref[i:window_steps + i], r_q=r_q)
        next = next_com(jerk=jerk[0], previous=prev, t_step=t_step)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h_com / g]) @ next)
        prev = next
        #Possible trajectory
        # possible_com_trajectory = [next[0]]
        # for k in range(1, window_steps):
        #     next_k = next_com(jerk=jerk[k], previous=prev, t_step=t_step)
        #     possible_com_trajectory.append(next_k[0])
        # possible_trajectories.append(possible_com_trajectory)
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, x



def simulation_qp(t_step, steps, g, h_com, xk_init, zk_min, zk_max):
    '''
    :param t_step:
    :param steps:
    :param g:
    :param h_com:
    :param xk_init:
    :param support_values:[min_val_left, max_val_left, min_val_right, max_val_right]
    :return:
    '''
    # zk_min, zk_max = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
    #                                      duration_right=0.07, duration_transition=0.018,
    #                                      min_val_left=support_values[0], max_val_left=support_values[1],
    #                                      min_val_right=support_values[2], max_val_right=support_values[3])
    com = []
    com_velocity = []
    com_acceleration = []
    # If we choose to consider that the referentiel is the body referential, world_position contains the world position
    world_position = []
    cop = []
    prev = xk_init
    window_steps = 300

    zk_min = np.array(list(zk_min) + [zk_min[-1]] * window_steps)
    zk_max = np.array(list(zk_max) + [zk_max[-1]] * window_steps)

    # Construction of reused matrices for performance
    Pu = p_u_matrix(t_step, h_com, g, window_steps)
    Px = p_x_matrix(t_step, h_com, g, window_steps)
    time = np.arange(0, 9, t_step)
    for i in range(steps):
        jerk = optimal_jerk_qp(n=window_steps, xk_init=prev,
                               zk_min=zk_min[i:window_steps + i], zk_max=zk_max[i:window_steps + i],
                               Pu=Pu, Px=Px)
        next = next_com(jerk=jerk[0], previous=prev, t_step=t_step)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h_com / g]) @ next)
        prev = next
        ################## Debug
        if i % 100 == 0:

            cop_x_s, com_x_s = [], []
            cop_y_s, com_y_s = [], []
            jerk_x, jerk_y = [], []
            for k in range(window_steps):
                # Get the next x and y
                next_x = next_com(jerk=jerk[k], previous=prev, t_step=t_step)
                com_x_s.append(next_x[0])
                cop_x_s.append(np.array([1, 0, -h_com / g]) @ next_x)
                # jerk_x.append(jerks[k])
            plt.plot(cop_x_s, label="cop_x", color="green")
            plt.plot(jerk_x, label="jerk_x", color="red", linewidth="0.8")
            plt.plot(zk_max[i:window_steps + i], label="zk_max", color="green", linewidth="0.4")
            plt.plot(zk_min[i:window_steps + i], label="zk_min", color="green", linewidth="0.4")
            plt.title("QP" + str(i + 1))
            # plt.ylim((-2, 2))
            plt.legend()
            plt.show()

    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, time


def simulation_qp_perturbations(t_step, steps, g, h_com, r_q, xk_init, inst_perturbation, acc_perturbation,
                                zk_min, zk_max):
    com = []
    com_velocity = []
    com_acceleration = []
    cop = []
    prev = xk_init
    window_steps = 300

    zk_min = np.array(list(zk_min) + [zk_min[-1]] * window_steps)
    zk_max = np.array(list(zk_max) + [zk_max[-1]] * window_steps)

    # Construction of reused matrices for performance
    Pu = p_u_matrix(t_step, h_com, g, window_steps)
    Px = p_x_matrix(t_step, h_com, g, window_steps)
    for i in range(steps):
        jerk = optimal_jerk_qp(n=window_steps, xk_init=prev,
                               zk_min=zk_min[i:window_steps + i], zk_max=zk_max[i:window_steps + i],
                               Pu=Pu, Px=Px)
        next = next_com(jerk=jerk[0], previous=prev, t_step=t_step)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h_com / g]) @ next)
        # perturbation after (2.5 seconds)
        if i == int(inst_perturbation/t_step):
            # 2.5 seconds we add an impact, it changes the acceleration
            next[2] += acc_perturbation
        prev = next
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, x


def simulation_with_perturbations(t_step, steps, g, h_com, r_q, xk_init, inst_perturbation, acc_perturbation,
                                  zk_min, zk_max):

    window_steps = 300
    zk_min = np.array(list(zk_min) + [zk_min[-1]] * window_steps)
    zk_max = np.array(list(zk_max) + [zk_max[-1]] * window_steps)

    zk_ref = (zk_min + zk_max)/2
    com = []
    com_velocity = []
    com_acceleration = []
    cop = []
    prev = xk_init

    for i in range(steps):
        jerk = optimal_jerk(t_step=t_step, h_com=h_com, g=g, n=window_steps, xk_init=prev,
                            zk_ref=zk_ref[i:window_steps + i], r_q=r_q)
        next = next_com(jerk=jerk[0], previous=prev, t_step=t_step)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        copInter = np.array([1, 0, -h_com / g]) @ next
        cop.append(np.array([1, 0, -h_com / g]) @ next)
        # perturbation after (2.5 seconds)
        if i == int(inst_perturbation/t_step):
            # 2.5 seconds we add an impact, it changes the acceleration
            next[2] += acc_perturbation
        prev = next
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, x


def simulation_qp_coupled(simulation_time, prediction_time, T_pred, T_control, h, g, alpha, gamma, xk_init, yk_init,
                          zk_ref_x, zk_ref_y, theta_ref, foot_dimensions):
    """
    Run the MPC to generate cop that allows stable walking
    :param simulation_time: total duration of the simulation
    :param prediction_time: time for prediction horizon
    :param T_pred: sampling time for trajectory prediction
    :param T_control: sampling time to run the MPC
    :param h: COM height for the robot
    :param g: gravitational constant
    :param alpha: minimization weight for the jerk
    :param gamma: minimzation weight for the references
    :param xk_init: initial (position, velocity, acceleration) in x
    :param yk_init: initial (position, velocity, acceleration) in y
    :param zk_ref_x: step references for x
    :param zk_ref_y: step references for y
    :param theta_ref : foot orientation references
    :param foot_dimensions: length (x), width (y)
    :return:
    """
    # Problem matrices
    N = int(prediction_time / T_pred)
    Pzu = p_u_matrix(T_pred, h, g, N)
    Pzs = p_x_matrix(T_pred, h, g, N)
    Qprime = alpha * np.eye(N) + gamma * Pzu.T @ Pzu
    Q = np.block([[Qprime, np.zeros(shape=(N, N))], [np.zeros(shape=(N, N)), Qprime]])

    # Outputs
    com_x, com_y = [], []
    com_velocity_x, com_velocity_y = [], []
    com_acceleration_x, com_acceleration_y = [], []
    cop_x, cop_y = [], []
    prev_x, prev_y = xk_init, yk_init

    # Padding: To avoid that the last window lacks elements
    zk_ref_x = np.array(list(zk_ref_x) + [zk_ref_x[-1]] * int(prediction_time / T_control))
    zk_ref_y = np.array(list(zk_ref_y) + [zk_ref_y[-1]] * int(prediction_time / T_control))
    theta_ref = np.array(list(theta_ref) + [theta_ref[-1]] * int(prediction_time / T_control))

    # Run the simulation
    for i in range(int(simulation_time / T_control)):
        # Get the current prediction horizon
        zk_ref_pred_x = zk_ref_x[i:i + int(prediction_time / T_control)]
        zk_ref_pred_x = zk_ref_pred_x[::int(T_pred / T_control)]  # Down sampling
        assert (len(zk_ref_pred_x) == N)

        zk_ref_pred_y = zk_ref_y[i:i + int(prediction_time / T_control)]
        zk_ref_pred_y = zk_ref_pred_y[::int(T_pred / T_control)]  # Down sampling
        assert (len(zk_ref_pred_y) == N)

        theta_ref_pred = theta_ref[i:i + int(prediction_time / T_control)]
        theta_ref_pred = theta_ref_pred[::int(T_pred / T_control)]  # Down sampling
        assert (len(theta_ref_pred) == N)

        # Solve the optimization problem ove the current prediction horizon
        p = gamma * np.hstack((Pzu.T @ (Pzs @ prev_x - zk_ref_pred_x), Pzu.T @ (Pzs @ prev_y - zk_ref_pred_y)))

        D = Dk_matrix(N, theta_ref_pred)
        b = np.array(
            [foot_dimensions[0] / 2, foot_dimensions[0] / 2, foot_dimensions[1] / 2, foot_dimensions[1] / 2] * N)
        G = D @ np.block([[Pzu, np.zeros(shape=(N, N))], [np.zeros(shape=(N, N)), Pzu]])
        h_cond = b + D @ np.hstack((zk_ref_pred_x - Pzs @ prev_x, zk_ref_pred_y - Pzs @ prev_y))

        jerk = solve_qp(P=Q, q=p, G=G, h=h_cond, solver="quadprog")
        if jerk is None:
            print(f"Cannot solve the QP at iteration {i}, most likely the value of xk diverges")
            return
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
        next_x, next_y = next_com(jerk=jerk[0], previous=prev_x, t_step=T_pred), \
                         next_com(jerk=jerk[N], previous=prev_y, t_step=T_pred)
        com_x.append(next_x[0])
        com_y.append(next_y[0])
        com_velocity_x.append(next_x[1])
        com_velocity_y.append(next_y[1])
        com_acceleration_x.append(next_x[2])
        com_acceleration_y.append(next_y[2])
        cop_x.append(np.array([1, 0, -h / g]) @ next_x)
        cop_y.append(np.array([1, 0, -h / g]) @ next_y)
        # Update the status of the position
        prev_x, prev_y = next_x, next_y



    return cop_x, com_x, cop_y, com_y


