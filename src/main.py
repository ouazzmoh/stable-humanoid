import numpy as np


def p_u_matrix(t_step, h, g, n):
    """ Calculate the P_u matrix.
        The matrix is used to solve the QP problem (equation 11)
        :param t_step: the time step
        :param h: the altitude of the center of mass
        :param g: norm of gravity
        :param n: number of steps in the QP problem
    """
    p_u = np.zeros(shape=(n, n))
    for diagonal_index in range(n):
        # We loop over the lower diagonals to fill the toeplitz matrix
        fill_value = (diagonal_index + 1) * (t_step**3)/6 - t_step * h /g
        if diagonal_index == 0:
            np.fill_diagonal(p_u, fill_value)
        else:
            np.fill_diagonal(p_u[diagonal_index:, :-diagonal_index], fill_value)
    return p_u


def p_x_matrix(t_step, h, g, n):
    """ Calculate the P_x matrix.
            The matrix is used to solve the QP problem (equation 11)
            :param t_step: the time step
            :param h: the altitude of the center of mass
            :param g: norm of gravity
            :param n: number of steps in the QP problem
    """

    p_x = np.ones(shape=(n, 3))
    for i in range(n):
        # The first column is already set to ones
        p_x[i][1] = (i+1) * t_step
        p_x[i][2] = (i+1) * (t_step**2)/2 - h/g
    return p_x


def optimal_jerk(t_step, h_com, g, n, xk_init, zk_ref, r_q):
    p_u = p_u_matrix(t_step, h_com, g, n)
    p_x = p_x_matrix(t_step, h_com, g, n)
    result = - np.linalg.inv(p_u.T @ p_u + r_q * np.eye(n)) @ p_u.T @ (p_x @ xk_init - zk_ref)
    return result


def next_com(jerk, previous, t_step):
    # previous :array of (position, velocity, acceleration)
    # first matrix of equation a
    a = np.zeros(shape=(3, 3))
    np.fill_diagonal(a, 1)
    np.fill_diagonal(a[1:, :-1], t_step)
    a[-1, 0] = (t_step**2) / 2
    # first matrix of equation b
    b = np.array([(t_step**3)/6, (t_step**2)/2, t_step])
    return a @ previous + b * jerk[0]




def precompute_trajectory(t_step, h_com, g, n, xk_init, zk_ref):
    jerk_opt = optimal_jerk(t_step, h_com, g, n, xk_init, zk_ref)


def mid(arr): return abs(arr[1] - arr[0])/2


def main():
    # Constructing z_ref
    # the convex hull when the support is in the left
    z_left_single = np.array([-0.17, -0.04])
    z_left_ref = mid(z_left_single)
    # the convex hull when the support is in the right
    z_right_single = np.array([0.04, 0.17])
    z_right_ref = mid(z_right_single)
    # the convex hull when the support is double
    z_double = np.array([-0.17, 0.17])
    z_double_ref = mid(z_double)
    steps = 300
    zk_ref = np.zeros(steps)
    for i in range(steps):
        zk_ref[i] = z_double_ref
    xk_init = np.zeros(3)
    jerk = optimal_jerk(t_step=5, h_com=0.8, g=9.81, n=steps, xk_init=xk_init, zk_ref=zk_ref, r_q=1e-6)
    prev = xk_init
    for i in range(30):
        next = next_com(jerk=jerk, previous=prev, t_step=5)
        print(next)
        prev = next



if __name__ == "__main__":
    main()




