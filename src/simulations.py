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


def simulation_with_feedback(t_step, steps, g, h_com, r_q, xk_init):
    zk_min, zk_max = construct_zmin_zmax(steps=steps,duration_double_init=0.26, duration_left=0.07,
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
        # possible_com_trajectory = [next[0]]
        # for k in range(1, window_steps):
        #     next_k = next_com(jerk=jerk[k], previous=prev, t_step=t_step)
        #     possible_com_trajectory.append(next_k[0])
        # possible_trajectories.append(possible_com_trajectory)
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, x



def simulation_qp(t_step, steps, g, h_com, xk_init, support_values):
    '''
    :param t_step:
    :param steps:
    :param g:
    :param h_com:
    :param xk_init:
    :param support_values:[min_val_left, max_val_left, min_val_right, max_val_right]
    :return:
    '''
    zk_min, zk_max = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
                                         duration_right=0.07, duration_transition=0.018,
                                         min_val_left=support_values[0], max_val_left=support_values[1],
                                         min_val_right=support_values[2], max_val_right=support_values[3])
    com = []
    com_velocity = []
    com_acceleration = []
    cop = []
    prev = xk_init
    window_steps = 300

    # Construction of reused matrices for performance
    Pu = p_u_matrix(t_step, h_com, g, window_steps)
    Px = p_x_matrix(t_step, h_com, g, window_steps)
    for i in range(steps - window_steps):
        jerk = optimal_jerk_qp(n=window_steps, xk_init=prev,
                               zk_min=zk_min[i:window_steps + i], zk_max=zk_max[i:window_steps + i],
                               Pu=Pu, Px=Px)
        next = next_com(jerk=jerk[0], previous=prev, t_step=t_step)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h_com / g]) @ next)
        prev = next
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, x


def simulation_qp_perturbations(t_step, steps, g, h_com, r_q, xk_init, inst_perturbation, acc_perturbation):
    zk_min, zk_max = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
                                         duration_right=0.07, duration_transition=0.018,
                                         min_val_left=-0.13, max_val_left=-0.07,
                                         min_val_right=0.07, max_val_right=0.13)
    com = []
    com_velocity = []
    com_acceleration = []
    cop = []
    prev = xk_init
    window_steps = 300

    # Construction of reused matrices for performance
    Pu = p_u_matrix(t_step, h_com, g, window_steps)
    Px = p_x_matrix(t_step, h_com, g, window_steps)
    for i in range(steps - window_steps):
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


def simulation_with_perturbations(t_step, steps, g, h_com, r_q, xk_init, inst_perturbation, acc_perturbation):
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
    for i in range(steps - window_steps):
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


def simulation_x_y(t_step, steps, g, h_com, xk_init, yk_init):
    zk_x_min, zk_x_max = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
                                         duration_right=0.07, duration_transition=0.018,
                                         min_val_left=-0.13, max_val_left=-0.07,
                                         min_val_right=0.07, max_val_right=0.13)
    zk_y_min, zk_y_max = construct_zmin_zmax(steps=steps, duration_double_init=0.26, duration_left=0.07,
                                             duration_right=0.07, duration_transition=0.018,
                                             min_val_left=-0.3, max_val_left=0.3,
                                             min_val_right=-0.3, max_val_right=0.3)
    com_x = []
    com_velocity_x = []
    com_acceleration_x = []
    cop_x = []
    prev_x = xk_init

    com_y = []
    com_velocity_y = []
    com_acceleration_y = []
    cop_y = []
    prev_y = xk_init

    window_steps = 300

    # Construction of reused matrices for performance
    Pu = p_u_matrix(t_step, h_com, g, window_steps)
    Px = p_x_matrix(t_step, h_com, g, window_steps)
    for i in range(steps - window_steps):
        jerk_x = optimal_jerk_qp(n=window_steps, xk_init=prev_x,
                               zk_min=zk_x_min[i:window_steps + i], zk_max=zk_x_max[i:window_steps + i],
                               Pu=Pu, Px=Px)
        next_x = next_com(jerk=jerk_x[0], previous=prev_x, t_step=t_step)
        com_x.append(next_x[0])
        com_velocity_x.append(next_x[1])
        com_acceleration_x.append(next_x[2])
        cop_x.append(np.array([1, 0, -h_com / g]) @ next_x)
        prev_x = next_x
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_x_min, zk_x_max, x