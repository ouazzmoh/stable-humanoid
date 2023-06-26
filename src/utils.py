import numpy as np
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


def construct_zmin_zmax_moving2(steps, duration_double_init, duration_step, duration_transition,
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

def u_matrix(N, steps_x, steps_y, occ_dict):
    assert(len(steps_y) == len(steps_x))
    U = np.zeros(shape=(N, len(steps_x)))
    # Current position in the U matrix
    position = 0
    for i in range(len(steps_x)):
        # Fill rows according to the occurrence of each step
        occ = occ_dict[(steps_x[i], steps_y[i])]
        for k in range(occ):
            U[position + k, i] = 1
        # Update position
        position += occ

    return U





