import numpy as np
from scipy.special import binom
import os
import h5py as h5
from qpsolvers import solve_qp


def p_z_u_matrix(t_step, h, g, n):
    """ Calculate the P_u matrix.
        The matrix is used to solve the QP problem (equation 11)
        :param t_step: the time step
        :param h: the altitude of the center of mass
        :param g: norm of gravity
        :param n: number of steps in the QP problem
    """
    Pu = np.zeros(shape=(n, n))
    for diagonal_index in range(n):
        # We loop over the lower diagonals to fill the toeplitz matrix
        if diagonal_index == 0:
            np.fill_diagonal(Pu, (t_step**3)/6 - t_step * h /g)
        else:
            fill_value = (1 + 3*diagonal_index + 3 * (diagonal_index**2)) * (t_step ** 3) / 6 - t_step * h / g
            np.fill_diagonal(Pu[diagonal_index:, :-diagonal_index], fill_value)
    return Pu


def p_z_s_matrix(t_step, h, g, n):
    """ Calculate the P_x matrix.
            The matrix is used to solve the QP problem (equation 11)
            :param t_step: the time step
            :param h: the altitude of the center of mass
            :param g: norm of gravity
            :param n: number of steps in the QP problem
    """

    Px = np.ones(shape=(n, 3))
    for i in range(n):
        # The first column is already set to ones
        Px[i][1] = (i+1) * t_step
        Px[i][2] = ((i+1)**2) * (t_step**2)/2 - h/g
    return Px


def p_v_u_matrix(t_step, n):
    """
    Pvu matrix
    """
    Pvu = np.zeros(shape=(n, n))
    for diagonal_index in range(n):
        # We loop over the lower diagonals to fill the toeplitz matrix
        if diagonal_index == 0:
            np.fill_diagonal(Pvu, (t_step**2)/2)
        else:
            fill_value = (1 + 2*diagonal_index) * (t_step ** 2) / 2
            np.fill_diagonal(Pvu[diagonal_index:, :-diagonal_index], fill_value)
    return Pvu


def p_v_s_matrix(t_step, n):
    """
    Pvs matrix
    """

    Pvs = np.zeros(shape=(n, 3))
    for i in range(n):
        # The first column is already set to ones
        Pvs[i][1] = 1
        Pvs[i][2] = (i+1) * t_step
    return Pvs


def optimal_jerk(t_step, h_com, g, n, xk_init, zk_ref, r_q):
    """
    Solve the QP problem analytically
    :param t_step:
    :param h_com:
    :param g:
    :param n:
    :param xk_init:
    :param zk_ref:
    :param r_q:
    :return:
    """
    Pu = p_z_u_matrix(t_step, h_com, g, n)
    Px = p_z_s_matrix(t_step, h_com, g, n)
    # result = - np.linalg.inv(p_u.T @ p_u + r_q * np.eye(n)) @ p_u.T @ (p_x @ xk_init - zk_ref)
    result = np.linalg.solve(Pu.T @ Pu + r_q * np.eye(n),  -Pu.T @ (Px @ xk_init - zk_ref))
    return result


def optimal_jerk_qp(n, xk_init, zk_min, zk_max, Pu, Px):
    # Objective matrix Q (Quadratic part)
    Q = np.eye(n)
    # Objective vector p (Linear part)
    p = np.zeros(n)
    # Inequality constraints Gx <= h
    G = np.vstack((Pu, -Pu))  # double stack Pu to handle both upper and lower bound constraints
    epsilon = 1e-3 * np.ones(n)  # using epsilon in the constraints to make the inequality strict
    h = np.hstack((zk_max - Px @ xk_init - epsilon, -(zk_min - Px @ xk_init) - epsilon))  # same here for h
    # Solving the QP problem
    x = solve_qp(Q, p, G, h, solver="quadprog")
    return x


def d_x_vector(theta):
    """
    Coordinates of the normal vectors to the edges of the feet (x)
    :param theta:
    :return:
    """
    return np.array([np.cos(theta), -np.cos(theta), -np.sin(theta), np.sin(theta)])


def d_y_vector(theta):
    """
    Coordinates of the normal vectors to the edges of the feet (y)
    :param theta:
    :return:
    """
    return np.array([np.sin(theta), -np.sin(theta), np.cos(theta), -np.cos(theta)])

def Dk_matrix(N, theta_ref):
    Dx = np.zeros(shape=(4 * N, N))
    Dy = np.zeros(shape=(4 * N, N))
    for j in range(N):
        d_x = d_x_vector(theta_ref[j])
        d_y = d_y_vector(theta_ref[j])
        for k in range(4):
            Dx[4 * j + k, j] = d_x[k]
            Dy[4 * j + k, j] = d_y[k]
    return np.hstack([Dx, Dy])



def optimal_jerk_qp_2D(n, xk_init, yk_init, zk_ref_x, zk_ref_y,  Pzu, Pzs, alpha, gamma, theta_ref, foot_dimensions, Q):
    """
    :param n:
    :param xk_init:
    :param yk_init:
    :param zk_ref_x:
    :param zk_ref_y:
    :param Pzu:
    :param Pzs:
    :param alpha:
    :param gamma:
    :param theta_ref:
    :param foot_dimensions: (length x width)
    :return:
    """
    # Objective matrix Q (Quadratic part)
    # Qprime = alpha * np.eye(n) + gamma * Pzu.T @ Pzu
    # Q = np.block([[Qprime, np.zeros(shape=(n, n))]
    #              , [np.zeros(shape=(n, n)), Qprime]])
    # Objective vector p (Linear part)
    p = np.hstack([gamma * Pzu.T @ (Pzs @ xk_init - zk_ref_x), gamma * Pzu.T @ (Pzs @ yk_init - zk_ref_y)])
    # Inequality constraints Gx <= h
    Dk = Dk_matrix(n, theta_ref)
    Pzu_Pzu = np.block([[Pzu, np.zeros(shape=(n, n))],
                       [np.zeros(shape=(n, n)), Pzu]])
    G = Dk @ Pzu_Pzu
    # b is in the form (length/2, -length/2, length/2, -........., width/2, -width/2, width/2,....)
    b_k = np.array([foot_dimensions[0]/2, foot_dimensions[0]/2]*n + [foot_dimensions[1]/2, foot_dimensions[1]/2]*n)
    h = b_k + Dk @ np.hstack([zk_ref_x - Pzs @ xk_init, zk_ref_y - Pzs @ yk_init])
    # Solving the QP problem
    u = solve_qp(Q, p, G, h, solver="quadprog")
    return u


def next_com(jerk, previous, t_step):
    """
    Getting the next (position, velocity, acceleration) vector using the recursive
    integration scheme
    :param jerk:
    :param previous:
    :param t_step:
    :return:
    """
    # previous :array of (position, velocity, acceleration)
    # first matrix of equation a
    A = np.zeros(shape=(3, 3))
    np.fill_diagonal(A, 1)
    np.fill_diagonal(A[:(1+1), 1:], t_step)
    A[0, -1] = (t_step**2) / 2
    # first matrix of equation b
    B = np.array([(t_step**3)/6, (t_step**2)/2, t_step])
    return A @ previous + B * jerk


def mid(arr): return (arr[0] + arr[1])/2




def construct_zmin_zmax_with_double(steps, duration_double_init, duration_left, duration_right, duration_transition,
                        min_val_left, max_val_left, min_val_right, max_val_right):
    """
        Construct the minimum and maximum for the center of pressure
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
        :return: two arrays z_min and z_max
        """
    # Constructing z_ref
    # the convex hull when the support is in the left
    z_left_single = np.array([min_val_left, max_val_left])
    # z_left_ref = mid(z_left_single)
    # the convex hull when the support is in the right
    z_right_single = np.array([min_val_right, max_val_right])
    # z_right_ref = mid(z_right_single)
    # the convex hull when the support is double
    z_double = np.array([min_val_left, max_val_right])
    # z_double_ref = mid(z_double)
    # steps = 300
    begin_min = [z_double[0]] * int(steps * duration_double_init)
    begin_max = [z_double[1]] * int(steps * duration_double_init)
    left_min = [z_left_single[0]] * int(steps * duration_left) + [z_double[0]] * int(steps * duration_transition)
    left_max = [z_left_single[1]] * int(steps * duration_left) + [z_double[1]] * int(steps * duration_transition)
    right_min = [z_right_single[0]] * int(steps * duration_right) + [z_double[0]] * int(steps * duration_transition)
    right_max = [z_right_single[1]] * int(steps * duration_right) + [z_double[1]] * int(steps * duration_transition)
    zk_min = begin_min + (left_min + right_min) * 3
    zk_min += [z_double[0]] * abs((steps - len(zk_min)))
    zk_max = begin_max + (left_max + right_max) * 3
    zk_max += [z_double[1]] * abs((steps - len(zk_max)))
    return np.array(zk_min), np.array(zk_max)



def construct_zmin_zmax(steps, duration_double_init, duration_step,
                        foot_size, spacing):
    """
        Construct the minimum and maximum for the center of pressure
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
        :return: two arrays z_min and z_max
        """

    # Initial double support
    zk_min = [-(foot_size + spacing)] * int(duration_double_init*steps)
    zk_max = [foot_size + spacing] * int(duration_double_init*steps)
    # Number of steps to take
    periods = int((steps - len(zk_min)) / (duration_step * steps * 2)) - 1
    # First period of steps
    left_min = [-(foot_size + spacing)] * int(steps * duration_step)
    left_max = [0] * int(steps * duration_step)
    right_min = [0] * int(steps * duration_step)
    right_max = [foot_size + spacing] * int(steps * duration_step)
    # Multiple periods
    zk_min += (left_min + right_min) * periods
    zk_max += (left_max + right_max) * periods
    zk_min += [-(foot_size+spacing)] * abs((steps - len(zk_min)))
    zk_max += [foot_size+spacing] * abs((steps - len(zk_max)))
    return np.array(zk_min), np.array(zk_max)

def construct_zmin_zmax_moving_with_double(steps, duration_double_init, duration_step, duration_transition,
                        foot_size, spacing):
    """
        The robot moves forward
        Construct the minimum and maximum for the center of pressure
        This is for a moving robot
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
        :return: two arrays z_min and z_max
        """

    # Initial double support
    zk_min = [-foot_size] * int(steps * duration_double_init)
    zk_max = [foot_size] * int(steps * duration_double_init)

    # Lifting foot first step
    zk_min += [-foot_size]*int(steps*duration_step)
    zk_max += [0] * int(steps*duration_step)

    # First Transition
    zk_min += [-foot_size] * int(steps * duration_transition)
    zk_max += [foot_size] * int(steps * duration_transition)

    # Number of steps to take
    number_of_steps = int((steps - len(zk_min))/((duration_step + duration_transition)*steps))

    for step_number in range(1, number_of_steps):
        # Lifting foot for a step
        zk_min += [(step_number - 1) * foot_size] * int(steps * duration_step)
        zk_max += [step_number * foot_size] * int(steps * duration_step)
        # Transition
        zk_min += [(step_number-1) * foot_size] * int(steps * duration_transition)
        zk_max += [(step_number+1) * foot_size] * int(steps * duration_transition)

    zk_min += [zk_min[-1]] * abs(steps - len(zk_min))
    zk_max += [zk_max[-1]] * abs(steps - len(zk_max))

    return np.array(zk_min), np.array(zk_max)


def construct_zmin_zmax_moving2_with_double(steps, duration_double_init, duration_step, duration_transition,
                        foot_size, spacing, duration_back):
    """
        The robot moves forward to a certain point, then moves backwards
        Construct the minimum and maximum for the center of pressure
        This is for a moving robot
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
               duration_back: the moment where the robot starts going backwards
        :return: two arrays z_min and z_max
        """

    # Initial double support
    zk_min = [-foot_size] * int(steps * duration_double_init)
    zk_max = [foot_size] * int(steps * duration_double_init)

    # Lifting foot first step
    zk_min += [-foot_size]*int(steps*duration_step)
    zk_max += [0] * int(steps*duration_step)

    # First Transition
    zk_min += [-foot_size] * int(steps * duration_transition)
    zk_max += [foot_size] * int(steps * duration_transition)

    # Number of steps to take
    number_of_steps = int((steps - len(zk_min))/((duration_step + duration_transition)*steps))

    for step_number in range(1, int(duration_back * number_of_steps)):
        # Lifting foot for a step
        zk_min += [(step_number - 1) * foot_size] * int(steps * duration_step)
        zk_max += [step_number * foot_size] * int(steps * duration_step)
        # Transition
        zk_min += [(step_number-1) * foot_size] * int(steps * duration_transition)
        zk_max += [(step_number+1) * foot_size] * int(steps * duration_transition)

    #The robot starts moving backwards
    for step_number in range(int(duration_back * number_of_steps), number_of_steps):
        if step_number == int(duration_back * number_of_steps):
            # Lifting foot for a step
            zk_min += [(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
            zk_max += [(number_of_steps-1 - step_number + 1) * foot_size] * int(steps * duration_step)
            # Transition
            zk_min += [(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_transition)
            zk_max += [(number_of_steps-1 - step_number + 1) * foot_size] * int(steps * duration_transition)
            # Transition
            zk_min += [(number_of_steps - 1 - step_number - 1) * foot_size] * int(steps * duration_transition)
            zk_max += [(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_transition)
        else :
            # Lifting foot for a step
            zk_min += [(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
            zk_max += [(number_of_steps-1 - step_number+1) * foot_size] * int(steps * duration_step)
            # Transition
            zk_min += [(number_of_steps - 1 - step_number-1) * foot_size] * int(steps * duration_transition)
            zk_max += [(number_of_steps-1 - step_number+1) * foot_size] * int(steps * duration_transition)

    zk_min += [zk_min[-1]] * abs(steps - len(zk_min))
    zk_max += [zk_max[-1]] * abs(steps - len(zk_max))

    return np.array(zk_min), np.array(zk_max)



def construct_zmin_zmax_moving(steps, duration_double_init, duration_step, foot_size):
    """
        The robot moves forward
        Construct the minimum and maximum for the center of pressure
        This is for a moving robot
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
        :return: two arrays z_min and z_max
        """

    # Initial double support
    zk_min = [-foot_size] * int(steps * duration_double_init)
    zk_max = [foot_size] * int(steps * duration_double_init)

    # Step on other direction
    zk_min += [-foot_size]*int(steps*duration_step)
    zk_max += [foot_size] * int(steps*duration_step)

    # Number of steps to take
    number_of_steps = int((steps - len(zk_min))/(duration_step*steps)) - 1

    for step_number in range(1, number_of_steps):
        # Lifting foot for a step
        zk_min += [(step_number - 1) * foot_size] * int(steps * duration_step)
        zk_max += [step_number * foot_size] * int(steps * duration_step)

    zk_min += [zk_min[-1]] * abs(steps - len(zk_min))
    zk_max += [zk_max[-1]] * abs(steps - len(zk_max))

    return np.array(zk_min), np.array(zk_max)



def construct_zmin_zmax_sin(steps, duration_double_init, duration_step, duration_transition,
                        foot_size, spacing, number_of_periods):
    """
        The robot moves forward to a certain point, then moves backwards
        Construct the minimum and maximum for the center of pressure
        This is for a moving robot
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
               duration_back: the moment where the robot starts going backwards
        :return: two arrays z_min and z_max
        """

    # Initial double support
    zk_max = [foot_size] * int(steps * duration_double_init)
    zk_min = [-foot_size] * int(steps * duration_double_init)

    zk_min_up, zk_max_up = construct_half_period_up(int(steps * (1/(2*number_of_periods))), foot_size, duration_step,
                                                    duration_transition)

    zk_min_down, zk_max_down = construct_half_period_down(int(steps * (1/(2*number_of_periods))), foot_size, duration_step,
                                                          duration_transition)


    for _ in range(number_of_periods):
        zk_min += zk_min_up + zk_min_down
        zk_max += zk_max_up + zk_max_down
    zk_min += [zk_min[-1]] * abs(steps - len(zk_min))
    zk_max += [zk_max[-1]] * abs(steps - len(zk_max))

    return np.array(zk_min), np.array(zk_max)


def construct_zmin_zmax_cos(steps, duration_double_init, duration_step, duration_transition,
                        foot_size, spacing, number_of_periods):
    """
        The robot moves forward to a certain point, then moves backwards
        Construct the minimum and maximum for the center of pressure
        This is for a moving robot
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
               duration_back: the moment where the robot starts going backwards
        :return: two arrays z_min and z_max
        """

    # # Initial double support
    # zk_max = [foot_size] * int(steps * duration_double_init)
    # zk_min = [-foot_size] * int(steps * duration_double_init)


    zk_min_up, zk_max_up = construct_half_period_up(int(steps * (1/(2*number_of_periods))), foot_size, duration_step,
                                                    duration_transition)

    zk_min_down, zk_max_down = construct_half_period_down(int(steps * (1/(2*number_of_periods))), foot_size, duration_step,
                                                          duration_transition)

    zk_max = zk_max_up[len(zk_max_up)//2:]
    zk_min = zk_min_up[len(zk_min_up)//2:]

    zk_max = [zk_max[0]] * int(steps * duration_double_init) + zk_max
    zk_min = [zk_min[0] - foot_size] * int(steps * duration_double_init) + zk_min

    for _ in range(number_of_periods):
        zk_min += zk_min_down + zk_min_up
        zk_max += zk_max_down + zk_max_up
    zk_min += [zk_min[-1]] * abs(steps - len(zk_min))
    zk_max += [zk_max[-1]] * abs(steps - len(zk_max))

    return np.array(zk_min), np.array(zk_max)


def construct_half_period_down(steps, foot_size, duration_step, duration_transition):
    zk_min, zk_max = [], []
    # Lifting foot first step
    zk_max += [foot_size] * int(steps * duration_step)
    zk_min += [0] * int(steps * duration_step)

    # First Transition
    zk_max += [foot_size] * int(steps * duration_transition)
    zk_min += [-foot_size] * int(steps * duration_transition)

    # Number of steps to take
    number_of_steps = int((steps - len(zk_min)) / ((duration_step + duration_transition) * steps))

    for step_number in range(1, int(0.5 * number_of_steps)):
        # Lifting foot for a step
        zk_max += [-(step_number - 1) * foot_size] * int(steps * duration_step)
        zk_min += [-step_number * foot_size] * int(steps * duration_step)
        # Transition
        zk_max += [-(step_number - 1) * foot_size] * int(steps * duration_transition)
        zk_min += [-(step_number + 1) * foot_size] * int(steps * duration_transition)

    # The robot starts moving backwards
    for step_number in range(int(0.5 * number_of_steps), number_of_steps):
        if step_number == int(0.5 * number_of_steps):
            # Lifting foot for a step
            zk_max += [-(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
            zk_min += [-(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_step)
            # Transition
            zk_max += [-(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_transition)
            zk_min += [-(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_transition)
            # Transition
            zk_max += [-(number_of_steps - 1 - step_number - 1) * foot_size] * int(steps * duration_transition)
            zk_min += [-(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_transition)
        elif step_number == number_of_steps-1:
            # This function is mainly used for the sin and cos functions, and we don't want to stop (No double support)
            # between half periods, the transition will be accounted for in the beginning of the following references
            # Lifting foot for a step
            # Lifting foot for a step
            zk_max += [-(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
            zk_min += [-(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_step)

        else:
            # Lifting foot for a step
            zk_max += [-(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
            zk_min += [-(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_step)
            # Transition
            zk_max += [-(number_of_steps - 1 - step_number - 1) * foot_size] * int(steps * duration_transition)
            zk_min += [-(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_transition)


    return zk_min, zk_max



def construct_half_period_up(steps, foot_size, duration_step, duration_transition):

    zk_min, zk_max = [], []
    # Lifting foot first step
    zk_min += [-foot_size] * int(steps * duration_step)
    zk_max += [0] * int(steps * duration_step)

    # First Transition
    zk_min += [-foot_size] * int(steps * duration_transition)
    zk_max += [foot_size] * int(steps * duration_transition)

    # Number of steps to take
    number_of_steps = int((steps - len(zk_min)) / ((duration_step + duration_transition) * steps))

    for step_number in range(1, int(0.5 * number_of_steps)):
        # Lifting foot for a step
        zk_min += [(step_number - 1) * foot_size] * int(steps * duration_step)
        zk_max += [step_number * foot_size] * int(steps * duration_step)
        # Transition
        zk_min += [(step_number - 1) * foot_size] * int(steps * duration_transition)
        zk_max += [(step_number + 1) * foot_size] * int(steps * duration_transition)

    # The robot starts moving backwards
    for step_number in range(int(0.5 * number_of_steps), number_of_steps):
        if step_number == int(0.5 * number_of_steps):
            # Lifting foot for a step
            zk_min += [(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
            zk_max += [(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_step)
            # Transition
            zk_min += [(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_transition)
            zk_max += [(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_transition)
            # Transition
            zk_min += [(number_of_steps - 1 - step_number - 1) * foot_size] * int(steps * duration_transition)
            zk_max += [(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_transition)
        elif step_number == number_of_steps-1:
            # This function is mainly used for the sin and cos functions, and we don't want to stop (No double support)
            # between half periods, the transition will be accounted for in the beginning of the following references
            # Lifting foot for a step
            # Lifting foot for a step
            zk_min += [(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
            zk_max += [(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_step)
        else:
            # Lifting foot for a step
            zk_min += [(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
            zk_max += [(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_step)
            # Transition
            zk_min += [(number_of_steps - 1 - step_number - 1) * foot_size] * int(steps * duration_transition)
            zk_max += [(number_of_steps - 1 - step_number + 1) * foot_size] * int(steps * duration_transition)

    return zk_min, zk_max


def construct_zref(steps):
    """
    Construct the references for the center of pressure
    :param steps: number of steps to sample the horizon
    :return: array of z_ref
    """
    zk_min, zk_max = construct_zmin_zmax(steps)
    return (zk_min + zk_max)/2



def construct_speed_ref(steps, duration_double_init, stop_at, average_speed):
    speed = [0] * int(steps * duration_double_init)
    speed += [average_speed] * int(stop_at * steps)
    speed += [0] * (steps - len(speed))
    return np.array(speed)



def construct_zmin_zmax_moving2(steps, duration_double_init, duration_step,
                                foot_size, spacing, duration_back):
    """
        The robot moves forward to a certain point, then moves backwards
        Construct the minimum and maximum for the center of pressure
        This is for a moving robot
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
               duration_back: the moment where the robot starts going backwards
        :return: two arrays z_min and z_max
        """

    foot_size += spacing

    # Initial double support
    zk_min = [-foot_size] * int(steps * duration_double_init)
    zk_max = [foot_size] * int(steps * duration_double_init)

    # Lifting foot first step
    zk_min += [-foot_size]*int(steps*duration_step)
    zk_max += [0] * int(steps*duration_step)

    # Number of steps to take
    number_of_steps = int((steps - len(zk_min))/((duration_step)*steps))

    for step_number in range(1, int(duration_back * number_of_steps)):
        # Lifting foot for a step
        zk_min += [(step_number - 1) * foot_size] * int(steps * duration_step)
        zk_max += [step_number * foot_size] * int(steps * duration_step)

    #The robot starts moving backwards
    for step_number in range(int(duration_back * number_of_steps), number_of_steps):
        # Lifting foot for a step
        zk_min += [(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
        zk_max += [(number_of_steps-1 - step_number+1) * foot_size] * int(steps * duration_step)

    zk_min += [zk_min[-1]] * abs(steps - len(zk_min))
    zk_max += [zk_max[-1]] * abs(steps - len(zk_max))

    return np.array(zk_min), np.array(zk_max)

def get_file_path(filename):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    file = filename if filename.endswith(".hdf5") else f"{filename}.hdf5"
    file_path = os.path.join(current_dir, "../data", file)
    if os.path.exists(file_path):
        return file_path
    with h5.File(file_path, "w") as file:
        file.attrs["title"] = "sequence of walking MPC qp problems"
    return file_path

def store_qp_in_file(file :h5._hl.files.File, t: float, name: str, type: str, **kwargs):
    """Stores the qp problem matrices in the given file.

    Args:
        file (h5._hl.files.File): _description_
        t (float): time in seconds
        iter (int): will be used to name the qp problem in the file
    """
    group_name = f"{type}_problems"
    if group_name not in file:
        file.create_group(group_name)
    group = file[group_name]
    if name not in group:
        group.create_group(name)
    group = group[name]
    group.attrs["time"] = t
    for data_name, data in kwargs.items():
        if f"{data_name}" not in group:
            group.create_group(f"{data_name}")
        # data = kwargs[data_name]
        if f"{data_name}/data" not in group:
            group[data_name].create_dataset("data", 
                                            data=data,
                                            shape=data.shape,
                                            dtype=data.dtype,
                                            chunks=True,
                                            compression="gzip")
        else: #overwrite
            del group[f"{data_name}/data"]
            group[data_name].create_dataset("data", 
                                            data=data,
                                            shape=data.shape,
                                            dtype=data.dtype,
                                            chunks=True,
                                            compression="gzip")


def retrieve_problem_data_from_file(file: h5._hl.files.File, name: str, type: str) -> dict:
    """retrieves the qp problem from the given file

    Args:
        file: the hdf5 file to retrieve the problem from
        iter: 

    Raises:
        ValueError: if the problem was not found in the file

    Returns:
        dict: dictionnary that contains the problem data
    """
    qp_data = {}
    group_name = f"{type}_problems"
    if group_name not in file:
        raise ValueError("Problem not found")
    group = file[group_name]
    if name not in group:
        raise ValueError("Problem not found")
    group = group[name]
    for data_name in list(group.keys()):
        if f"{data_name}/data" in group:
            qp_data[data_name] = group[f"{data_name}/data"][:]
    return qp_data


from scipy.interpolate import UnivariateSpline

def smooth_data(data, smooth_factor=0):
    """Applies a spline smoothing to a 1D list.

    Arguments:
    data -- the list to smooth
    smooth_factor -- a factor controlling the amount of smoothing; higher values result in more smoothing

    Returns:
    A list containing the smoothed data.
    """
    x = list(range(len(data)))
    spl = UnivariateSpline(x, data, k=3, s=smooth_factor)
    smoothed_data = spl(x)
    return smoothed_data.tolist()


def remove_duplicates(lst):
    seen = []
    res = []
    for item in lst:
        if item is not None:
            # convert numpy array or list to tuple
            item_tuple = tuple(item.tolist()) if isinstance(item, np.ndarray) else tuple(item)
            if item_tuple not in seen:
                seen.append(item_tuple)
                res.append(tuple(item))
    return res

def Bernstein(n: int, k: int, x: float) -> float:
    """Compute the Bernstein polynomial.

    Given the parameters n, k, and x, this function calculates
    the value of the Bernstein polynomial at the given point x.

    Args:
        n: Degree of the polynomial.
        k: Index of the term.
        x: Point at which to evaluate the polynomial.

    Returns:
        float: Value of the Bernstein polynomial at x.

    """
    coeff = binom(n, k)
    return coeff * x ** k * (1 - x) ** (n - k)


def get_control_points(src: np.ndarray, dest: np.ndarray, dx: float=0.0, dy: float=0.0, dz : float=0.0) -> np.ndarray:
    """Compute control points for a Bezier curve interpolation.
.
    Given the source and destination points, this function calculates 
    the control points for a Bezier curve interpolation. The control 
    points are calculated by adding an offset to the source and destination points.

    Args:
        src: Source point coordinates.
        dest: Destination point coordinates.
        dx: Offset in the x direction (default: 0.0).
        dy: Offset in the y direction (default: 0.0).
        dz: Offset in the z direction (default: 0.0).

    Returns:
        np.ndarray: Array of control points for the Bezier curve.

    """
    P0, P3 = src, dest
    P1 = P0 + np.array([dx, dy, dz])
    P2 = P3 + np.array([dx, dy, dz]) 
    return np.vstack((P0, P1, P2, P3))


def get_mid(src: np.ndarray, dest: np.ndarray) -> np.ndarray:
    """Compute the midpoint between two arrays.

    Given two numpy arrays `src` and `dest`, this function calculates
    the midpoint by adding the elements of `src` and `dest` element-wise
    and dividing the result by 2.

    Args:
        src: The source point.
        dest: The destination point.

    Returns:
        np.ndarray: The computed midpoint array.

    """
    mid = src + dest / 2
    return mid


def group_not_none(lst):
    """
    Return the start and end indices of the groups of non-None values in the list.
    Args:
        lst:

    Returns:

    """
    start = None  # Start index of the current group
    groups = []  # List of groups
    for i, x in enumerate(lst):
        if x is not None and start is None:
            start = i  # Start a new group
        elif x is None and start is not None:
            groups.append((start, i - 1))  # End the current group
            start = None
    # If the list ended with a group, add it
    if start is not None:
        groups.append((start, len(lst) - 1))
    return groups