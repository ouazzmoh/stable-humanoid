import numpy as np
from matplotlib import pyplot as plt
import cvxpy as cp
from qpsolvers import solve_qp
import scipy.sparse as sp


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


def optimal_jerk_qp(t_step, h_com, g, n, xk_init, zk_min, zk_max, r_q, Pu, Px):
    # Objective matrix Q (Quadratic part)
    Q = np.eye(n)
    # Objective vector p (Linear part)
    p = np.zeros(n)
    # Inequality constraints Gx <= h
    G = np.vstack((Pu, -Pu))  # double stack Pu to handle both upper and lower bound constraints
    epsilon = 1e-3 * np.ones(n)  # using epsilon in the constraints to make the inequality strict
    h = np.hstack((zk_max - Px @ xk_init - epsilon, -(zk_min - Px @ xk_init) - epsilon))  # same here for h
    # Solving the QP problem
    x = solve_qp(Q, p, G, h, solver="osqp")
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



def construct_zmin_zmax(steps):
    """
        Construct the minimum and maximum for the center of pressure
        :param steps: number of steps to sample the horizon
        :return: array of z_ref
        """
    # Constructing z_ref
    # the convex hull when the support is in the left
    z_left_single = np.array([-0.13, -0.07])
    # z_left_ref = mid(z_left_single)
    # the convex hull when the support is in the right
    z_right_single = np.array([0.07, 0.13])
    # z_right_ref = mid(z_right_single)
    # the convex hull when the support is double
    z_double = np.array([-0.13, 0.13])
    # z_double_ref = mid(z_double)
    # steps = 300
    begin_min = [z_double[0]] * int(steps * 0.26)
    begin_max = [z_double[1]] * int(steps * 0.26)
    left_min = [z_left_single[0]] * int(steps * 0.07) + [z_double[0]] * int(steps * 0.01)
    left_max = [z_left_single[1]] * int(steps * 0.07) + [z_double[1]] * int(steps * 0.01)
    right_min = [z_right_single[0]] * int(steps * 0.07) + [z_double[0]] * int(steps * 0.01)
    right_max = [z_right_single[1]] * int(steps * 0.07) + [z_double[1]] * int(steps * 0.01)
    zk_min = begin_min + (left_min + right_min) * 3
    zk_min += [z_double[0]] * abs((steps - len(zk_min)) % steps)
    zk_max = begin_max + (left_max + right_max) * 3
    zk_max += [z_double[1]] * abs((steps - len(zk_max)) % steps)
    return np.array(zk_min), np.array(zk_max)


def construct_zref(steps):
    """
    Construct the references for the center of pressure
    :param steps: number of steps to sample the horizon
    :return: array of z_ref
    """
    zk_min, zk_max = construct_zmin_zmax(steps)
    return (zk_min + zk_max)/2

def simulation_no_feedback():
    steps = 300
    g = 9.81
    h_com = 0.8
    t_step = 5e-3
    r_q = 1e-6
    xk_init = (0, 0, 0)
    zk_ref = construct_zref(steps=steps)
    jerk = optimal_jerk(t_step=t_step, h_com=h_com, g=g, n=steps, xk_init=xk_init, zk_ref=zk_ref, r_q=r_q)
    com = []
    com_velocity = []
    com_acceleration = []
    cop = []
    prev = xk_init
    for i in range(steps):
        next = next_com(jerk=jerk[i], previous=prev, t_step=t_step)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h_com/g]) @ next)
        prev = next

    x = np.linspace(0, 9, steps)

    return cop, com, com_velocity, com_acceleration, zk_ref, x, jerk


def simulation_with_feedback():
    t_step = 5e-3
    # We simulate from 0 to 9 (s)
    steps = int(9/t_step)
    g = 9.81
    h_com = 0.8
    r_q = 1e-6
    xk_init = (0, 0, 0)
    zk_ref = construct_zref(steps=steps)
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
        for k in range(1, window_steps):
            next_k = next_com(jerk=jerk[k], previous=prev, t_step=t_step)
            possible_com_trajectory.append(next_k[0])
        possible_trajectories.append(possible_com_trajectory)
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_ref, x, possible_trajectories



def simulation_qp():
    t_step = 5e-3
    # We simulate from 0 to 9 (s)
    steps = int(9/t_step)
    g = 9.81
    h_com = 0.8
    r_q = 1e-6
    xk_init = (0, 0, 0)
    zk_min, zk_max = construct_zmin_zmax(steps=steps)
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
        jerk = optimal_jerk_qp(t_step=t_step, h_com=h_com, g=g, n=window_steps, xk_init=prev,
                               zk_min=zk_min[i:window_steps + i], zk_max=zk_max[i:window_steps + i],
                               r_q=r_q, Pu=Pu, Px=Px)
        next = next_com(jerk=jerk[0], previous=prev, t_step=t_step)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h_com / g]) @ next)
        prev = next
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, x


def simulation_qp_perturbations():
    t_step = 5e-3
    # We simulate from 0 to 9 (s)
    steps = int(9/t_step)
    g = 9.81
    h_com = 0.8
    r_q = 1e-6
    xk_init = (0, 0, 0)
    zk_min, zk_max = construct_zmin_zmax(steps=steps)
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
        jerk = optimal_jerk_qp(t_step=t_step, h_com=h_com, g=g, n=window_steps, xk_init=prev,
                               zk_min=zk_min[i:window_steps + i], zk_max=zk_max[i:window_steps + i],
                               r_q=r_q, Pu=Pu, Px=Px)
        next = next_com(jerk=jerk[0], previous=prev, t_step=t_step)
        com.append(next[0])
        com_velocity.append(next[1])
        com_acceleration.append(next[2])
        cop.append(np.array([1, 0, -h_com / g]) @ next)
        # perturbation after (2.5 seconds)
        if i == 500:
            # 2.5 seconds we add an impact, it changes the acceleration
            next[2] += 2
        prev = next
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, x
def simulation_with_perturbations():
    t_step = 5e-3
    t_end = 9
    # We simulate from 0 to 9 (s)
    steps = int(t_end/t_step)
    g = 9.81
    h_com = 0.8
    r_q = 1e-6
    xk_init = (0, 0, 0)
    zk_min, zk_max = construct_zmin_zmax(steps=steps)
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
        if i == 500:
            # 2.5 seconds we add an impact, it changes the acceleration
            next[2] += 2
        prev = next
    x = np.arange(0, 9, t_step)
    return cop, com, com_velocity, com_acceleration, zk_min, zk_max, x



def main():
    cop, com, _, _, zk_min, zk_max, x = simulation_with_perturbations()
    # x is used to show the proper scale in the x-axis
    x = x[:len(cop)]
    plt.plot(x, zk_min[:len(cop)], linestyle="--", linewidth=0.5, color="gray")
    plt.plot(x, zk_max[:len(cop)], linestyle="--", linewidth=0.5, color="gray")
    plt.plot(x, cop, color="green", label="cop")
    plt.plot(x, com, color="red", label="com")
    # for i in range(len(possible_trajectories)):
    #     if i == 500:
    #         possible = possible_trajectories[i]
    #         possible = [0.0] * i + possible + [0.0] * (len(x) - len(possible) - i)
    #         plt.plot(x, np.array(possible[:len(x)]), linewidth=1, color="orange")
    plt.ylim(-0.15, 0.15)
    plt.legend(loc='upper right')
    plt.title("Analytical resolution with perturbation of 2 m.s-2")
    plt.show()





if __name__=="__main__":
    main()




