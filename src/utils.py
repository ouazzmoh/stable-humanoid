import numpy as np
from qpsolvers import solve_qp


def p_u_matrix(t_step, h, g, n):
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


def p_x_matrix(t_step, h, g, n):
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
    Pu = p_u_matrix(t_step, h_com, g, n)
    Px = p_x_matrix(t_step, h_com, g, n)
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




def construct_zmin_zmax(steps, duration_double_init, duration_left, duration_right, duration_transition,
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


def construct_zmin_zmax_moving(steps, duration_double_init, duration_step, duration_transition,
                        foot_size, spacing):
    """
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


def construct_zref(steps):
    """
    Construct the references for the center of pressure
    :param steps: number of steps to sample the horizon
    :return: array of z_ref
    """
    zk_min, zk_max = construct_zmin_zmax(steps)
    return (zk_min + zk_max)/2










