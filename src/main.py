import numpy as np
from matplotlib import pyplot as plt


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
        if diagonal_index == 0:
            np.fill_diagonal(p_u, (t_step**3)/6 - t_step * h /g)
        else:
            fill_value = (1 + 3*diagonal_index + 3 * (diagonal_index**2)) * (t_step ** 3) / 6 - t_step * h / g
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
        p_x[i][2] = ((i+1)**2) * (t_step**2)/2 - h/g
    return p_x


def optimal_jerk(t_step, h_com, g, n, xk_init, zk_ref, r_q):
    """
    Cacl
    :param t_step:
    :param h_com:
    :param g:
    :param n:
    :param xk_init:
    :param zk_ref:
    :param r_q:
    :return:
    """
    p_u = p_u_matrix(t_step, h_com, g, n)
    p_x = p_x_matrix(t_step, h_com, g, n)
    result = - np.linalg.inv(p_u.T @ p_u + r_q * np.eye(n)) @ p_u.T @ (p_x @ xk_init - zk_ref)
    return result


def next_com(jerk, previous, t_step):
    # previous :array of (position, velocity, acceleration)
    # first matrix of equation a
    a = np.zeros(shape=(3, 3))
    np.fill_diagonal(a, 1)
    np.fill_diagonal(a[:(1+1), 1:], t_step)
    a[0, -1] = (t_step**2) / 2
    # first matrix of equation b
    b = np.array([(t_step**3)/6, (t_step**2)/2, t_step])
    return a @ previous + b * jerk


def mid(arr): return (arr[0] + arr[1])/2


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

    begin = [z_double_ref] * int(steps*0.26)
    left = [z_left_ref] * int(steps*0.07) + [z_double_ref] * int(steps*0.01)
    right = [z_right_ref] * int(steps*0.07) + [z_double_ref] * int(steps*0.01)
    zk_ref = begin + (left + right) * 3
    zk_ref += [z_double_ref] * abs((steps - len(zk_ref)) % steps)
    zk_ref = np.array(zk_ref)

    x = np.linspace(0, 9, 300)

    prev = np.zeros(3)
    y_com = np.zeros(300)
    acc_com = np.zeros(300)
    y_cop = np.zeros(300)
    jerk = optimal_jerk(t_step=5* 1e-3, h_com=0.8, g=9.81, n=steps, xk_init=prev, zk_ref=zk_ref, r_q=1e-6)

    acc_proj = np.zeros(shape=(steps, steps))
    for diagonal_index in range(steps):
        # We loop over the lower diagonals to fill the toeplitz matrix
        if diagonal_index == 0:
            np.fill_diagonal(acc_proj, 5)
        else:
            np.fill_diagonal(acc_proj[diagonal_index:, :-diagonal_index], 5)


    # plt.plot(x, acc_proj @ jerk)


    for i in range(300):
        next = next_com(jerk=jerk[i], previous=prev, t_step=5)
        next_cop = np.array([1, 0, -0.8/9.8]) @ next
        y_com[i] = next[0]
        acc_com[i] = next[2]
        y_cop[i] = next_cop
        prev = next


    # plt.plot(x, zk_ref)
    # plt.plot(x, acc_com)
    # plt.plot(x, y_com)
    plt.plot(x, (0.8/9.8)*acc_com)
    # plt.plot(x, y_cop)
    # plt.ylim(bottom=-0.12, top=0.12)
    # plt.legend(['jerk', 'zk_ref', 'y_com', 'y_cop'])
    plt.show()



if __name__ == "__main__":
    main()




