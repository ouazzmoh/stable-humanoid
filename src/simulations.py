from utils import *

#
# def simulation_no_feedback(steps, g, h_com, t_step, r_q, xk_init):
#     zk_ref = construct_zref(steps=steps)
#     jerk = optimal_jerk(t_step=t_step, h_com=h_com, g=g, n=steps, xk_init=xk_init, zk_ref=zk_ref, r_q=r_q)
#     com = []
#     com_velocity = []
#     com_acceleration = []
#     cop = []
#     prev = xk_init
#     for i in range(steps):
#         next = next_com(jerk=jerk[i], previous=prev, t_step=t_step)
#         com.append(next[0])
#         com_velocity.append(next[1])
#         com_acceleration.append(next[2])
#         cop.append(np.array([1, 0, -h_com/g]) @ next)
#         prev = next
#
#     x = np.linspace(0, 9, steps)
#
#     return cop, com, com_velocity, com_acceleration, zk_ref, x, jerk


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



def simulation_possible_trajectories(t_step, steps, g, h_com, r_q, xk_init):
    zk_min, zk_max = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
                                         duration_right=0.07, duration_transition=0.018,
                                         min_val_left=-0.13, max_val_left=-0.07,
                                         min_val_right=0.07, max_val_right=0.13)
    zk_ref = (zk_min + zk_max)/2
    com = []
    com_velocity = []
    com_acceleration = []
    cop = []
    prev = xk_init
    window_steps = 300
    possible_trajectories = []
    for i in range(steps - window_steps):
        jerk = optimal_jerk(t_step=t_step, h_com=h_com, g=g, n=window_steps, xk_init=prev,
                            zk_ref=zk_ref[i:window_steps + i], r_q=r_q)
        next = next_com(jerk=jerk[0], previous=prev, t_step=t_step)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h_com / g]) @ next)
        prev = next
        #Possible trajectory
        possible_com_trajectory = [next[0]]
        if i == 0:
            for k in range(1, window_steps):
                next_k = next_com(jerk=jerk[k], previous=prev, t_step=t_step)
                possible_com_trajectory.append(next_k[0])
            possible_trajectories.append(possible_com_trajectory)
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, x, possible_trajectories


def simulation_qp_coupled(t_step, steps, g, h, xk_init, yk_init, zk_ref_x, zk_ref_y,
                          alpha, gamma, theta_ref, foot_dimensions):
    window_steps = 300

    zk_ref_x = np.array(list(zk_ref_x) + [zk_ref_x[-1]] * window_steps)
    zk_ref_y = np.array(list(zk_ref_y) + [zk_ref_y[-1]] * window_steps)

    theta_ref = np.array(list(theta_ref) + [theta_ref[-1]] * window_steps)

    com_x = []
    com_velocity_x = []
    com_acceleration_x = []
    cop_x = []
    prev_x = xk_init

    com_y = []
    com_velocity_y = []
    com_acceleration_y = []
    cop_y = []
    prev_y = yk_init

    # Construction of reused matrices for performance
    n = window_steps
    Pzu = p_u_matrix(t_step, h, g, n)
    Pzs = p_x_matrix(t_step, h, g, n)
    Qprime = alpha * np.eye(n) + gamma * Pzu.T @ Pzu
    Q = np.block([[Qprime, np.zeros(shape=(n, n))]
                     , [np.zeros(shape=(n, n)), Qprime]])
    time = np.arange(0, 9, t_step)
    for i in range(steps):
        # Solve the problem and get u = (x, y)
        jerks = optimal_jerk_qp_2D(n=window_steps, xk_init=prev_x, yk_init=prev_y,
                                   zk_ref_x=zk_ref_x[i:window_steps + i], zk_ref_y=zk_ref_y[i:window_steps + i],
                                   Pzu=Pzu, Pzs=Pzs, alpha=alpha, gamma=gamma,
                                   theta_ref=theta_ref[i:window_steps + i],
                                   foot_dimensions=foot_dimensions, Q=Q)
        # Get the next x and y
        try:
            next_x = next_com(jerk=jerks[0], previous=prev_x, t_step=t_step)
            com_x.append(next_x[0])
            com_velocity_x.append(next_x[1])
            com_acceleration_x.append(next_x[2])
            cop_x.append(np.array([1, 0, -h / g]) @ next_x)
            prev_x = next_x

            # Get the next y
            next_y = next_com(jerk=jerks[window_steps], previous=prev_y, t_step=t_step)
            com_y.append(next_y[0])
            com_velocity_y.append(next_y[1])
            com_acceleration_y.append(next_y[2])
            cop_y.append(np.array([1, 0, -h / g]) @ next_y)
            prev_y = next_y
        except:
            print("i: --> ", i)
            print("Bug here")



    return cop_x, com_x, cop_y, com_y, time


