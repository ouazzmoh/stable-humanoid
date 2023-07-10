import matplotlib.patches as patches
import matplotlib.transforms as transforms
from typing import List


from simulations import *

def plot_foot_steps(ax, zk_ref_x, zk_ref_y, theta_ref, foot_dimensions, spacing) -> None:
    """
    Plot the footsteps of the robot
    :param ax:  matplotlib ax returned by subplots
    :param zk_ref_x: array of cop center references for x
    :param zk_ref_y: array of cop center references for y
    :param foot_dimensions:  (length (x), width (y))
    :param init : draw the initial double support if the bool is true, by default it is true
    :return:
    """
    # Combine the two arrays to create an array of points
    points = np.array([zk_ref_x, zk_ref_y, theta_ref]).T
    # Get unique rows,
    _, idx = np.unique(points, axis=0, return_index=True)
    unique_points = points[np.sort(idx)]
    # Now your x and y coordinates are the first and second columns of unique_points
    steps_x = unique_points[:, 0]
    steps_y = unique_points[:, 1]
    theta = unique_points[:, 2] * (180/np.pi)  # convert to degrees
    plt.plot(steps_x, steps_y, 'go', label="ref_center")
    # Single supports

    for x, y, angle in zip(steps_x[1:], steps_y[1:], theta[1:]):
        # Create a rectangle
        rect = patches.Rectangle((x - foot_dimensions[0] / 2, y - foot_dimensions[1] / 2),
                                 foot_dimensions[0], foot_dimensions[1],
                                 linewidth=0.7, edgecolor='grey', facecolor='none')

        # Create a rotation transform.
        rotation = transforms.Affine2D().rotate_deg_around(x, y, angle)
        # Add the transform to the rectangle. The order of the multiplication matters.
        rect.set_transform(rotation + ax.transData)
        # Add the rectangle to the plot
        ax.add_patch(rect)
    # Double supports
    # for x, y, angle in zip([steps_x[0], steps_x[-1]], [steps_y[0], steps_y[-1]], [theta[0], theta[-1]]):
        x, y, angle = steps_x[0], steps_y[0], theta[0]
        rect1 = patches.Rectangle((x - foot_dimensions[0] / 2, y - foot_dimensions[1] - spacing/2),
                                  foot_dimensions[0], foot_dimensions[1],
                                  linewidth=0.7, edgecolor='grey', facecolor='none')
        rect2 = patches.Rectangle((x - foot_dimensions[0] / 2, y + spacing/2),
                                  foot_dimensions[0], foot_dimensions[1],
                                  linewidth=0.7, edgecolor='grey', facecolor='none')

        # Create a rotation transform.
        rotation = transforms.Affine2D().rotate_deg_around(x, y, angle)
        # Add the transform to the rectangle. The order of the multiplication matters.
        rect1.set_transform(rotation + ax.transData)
        rect2.set_transform(rotation + ax.transData)
        ax.add_patch(rect1)
        ax.add_patch(rect2)
    ax.set_aspect('equal', adjustable='datalim')


def plot_foot_steps_single(ax, zk_ref_x, zk_ref_y, theta_ref, foot_dimensions, spacing) -> None:
    """
    Plot the footsteps of the robot
    :param ax:  matplotlib ax returned by subplots
    :param zk_ref_x: array of cop center references for x
    :param zk_ref_y: array of cop center references for y
    :param foot_dimensions:  (length (x), width (y))
    :param init : draw the initial double support if the bool is true, by default it is true
    :return:
    """
    # Combine the two arrays to create an array of points
    points = np.array([zk_ref_x, zk_ref_y, theta_ref]).T
    # Get unique rows,
    _, idx = np.unique(points, axis=0, return_index=True)
    unique_points = points[np.sort(idx)]
    # Now your x and y coordinates are the first and second columns of unique_points
    steps_x = unique_points[:, 0]
    steps_y = unique_points[:, 1]
    theta = unique_points[:, 2] * (180/np.pi)  # convert to degrees
    plt.plot(steps_x, steps_y, 'go', label="ref_center")
    # Single supports

    for x, y, angle in zip(steps_x, steps_y, theta):
        if abs(y) > 0:
            # Create a rectangle
            rect = patches.Rectangle((x - foot_dimensions[0] / 2, y - foot_dimensions[1] / 2),
                                     foot_dimensions[0], foot_dimensions[1],
                                     linewidth=0.7, edgecolor='grey', facecolor='none')

            # Create a rotation transform.
            rotation = transforms.Affine2D().rotate_deg_around(x, y, angle)
            # Add the transform to the rectangle. The order of the multiplication matters.
            rect.set_transform(rotation + ax.transData)
            # Add the rectangle to the plot
            ax.add_patch(rect)
    ax.set_aspect('equal', adjustable='datalim')

def plot_intermediate_states(i, prev_x, prev_y, prediction_time, T_pred, T, jerk, h, g, N, zk_ref_pred_x,
                             zk_ref_pred_y, theta_ref_pred, foot_dimensions) -> None:
    '''
    Plot intermediate solutions for the QP problem
    :param i: iteration
    :param prev_x: previous x position
    :param prev_y: previous y position
    :param prediction_time:
    :param T_pred: Timestep for prediction
    :param T: Current timestep
    :param jerk: Last solved QP's jerk
    :param h: altitude of COM
    :param g: gravitational constant
    :param N: QP problem size
    :param zk_ref_pred_x: x step references
    :param zk_ref_pred_y: y step references
    :param theta_ref_pred: orientation references
    :param foot_dimensions:
    :return:
    '''
    cop_x_s, com_x_s = [], []
    cop_y_s, com_y_s = [], []
    jerk_x, jerk_y = [], []
    prev_x_s, prev_y_s = prev_x, prev_y
    for k in range(int(prediction_time / T_pred)):
        if k == 0:
            Tk = T
        else:
            Tk = T_pred
        # Get the next x and y
        next_x_s = next_com(jerk=jerk[k], previous=prev_x_s, t_step=Tk)
        com_x_s.append(next_x_s[0])
        cop_x_s.append(np.array([1, 0, -h / g]) @ next_x_s)
        next_y_s = next_com(jerk=jerk[N + k], previous=prev_y_s, t_step=Tk)
        com_y_s.append(next_y_s[0])
        cop_y_s.append(np.array([1, 0, -h / g]) @ next_y_s)
        prev_x_s, prev_y_s = next_x_s, next_y_s
        # jerk_x.append(jerks[k])
    _, ax = plt.subplots()
    ax.plot(cop_x_s, cop_y_s, color="green")
    ax.plot(com_x_s, com_y_s, color="red")
    ax.plot(cop_x_s[0], cop_y_s[0], "ro", label="cop_sol_init")
    ax.plot(cop_x_s[-1], cop_y_s[-1], "bo", label="cop_sol_end")
    plot_foot_steps_single(ax, zk_ref_pred_x, zk_ref_pred_y,
                                   theta_ref_pred, foot_dimensions, spacing=0.4)
    plt.title("QP" + str(i + 1))
    # plt.ylim((-0.6, 0.6))
    plt.legend()
    plt.show()


def plot_intermediate_states_adapting(i, prev_x, prev_y, prediction_time, T_pred, T, solution, h, g, N, zk_ref_pred_x,
                             zk_ref_pred_y, theta_ref_pred, foot_dimensions, Uk, zk_ref_pred_x_2, zk_ref_pred_y_2) -> None:
    '''
    Plot intermediate solutions for the QP problem
    :param i: iteration
    :param prev_x: previous x position
    :param prev_y: previous y position
    :param prediction_time:
    :param T_pred: Timestep for prediction
    :param T: Current timestep
    :param jerk: Last solved QP's jerk
    :param h: altitude of COM
    :param g: gravitational constant
    :param N: QP problem size
    :param zk_ref_pred_x: x step references
    :param zk_ref_pred_y: y step references
    :param theta_ref_pred: orientation references
    :param foot_dimensions:
    :return:
    '''
    cop_x_s, com_x_s = [], []
    cop_y_s, com_y_s = [], []
    jerk_x, jerk_y = [], []
    prev_x_s, prev_y_s = prev_x, prev_y
    for k in range(int(prediction_time / T_pred)):
        if k == 0:
            Tk = T
        else:
            Tk = T_pred
        # Get the next x and y
        next_x_s = next_com(jerk=solution[k], previous=prev_x_s, t_step=Tk)
        com_x_s.append(next_x_s[0])
        cop_x_s.append(np.array([1, 0, -h / g]) @ next_x_s)
        next_y_s = next_com(jerk=solution[N + Uk.shape[1] + k], previous=prev_y_s, t_step=Tk)
        com_y_s.append(next_y_s[0])
        cop_y_s.append(np.array([1, 0, -h / g]) @ next_y_s)
        prev_x_s, prev_y_s = next_x_s, next_y_s
        # jerk_x.append(jerks[k])
    _, ax = plt.subplots()
    ax.plot(cop_x_s, cop_y_s, color="green")
    # ax.plot(com_x_s, com_y_s, color="red")
    ax.plot(cop_x_s[0], cop_y_s[0], "ro", label="cop_sol_init")
    ax.plot(cop_x_s[-1], cop_y_s[-1], "bo", label="cop_sol_end")
    plot_foot_steps_single(ax, zk_ref_pred_x, zk_ref_pred_y,
                                   theta_ref_pred, foot_dimensions, spacing=0.4)
    # plot_foot_steps_single(ax, zk_ref_pred_x_2, zk_ref_pred_y_2,
    #                        theta_ref_pred, foot_dimensions, spacing=0.4)
    plt.title("QP" + str(i + 1))
    # plt.ylim((-0.6, 0.6))
    plt.legend()
    plt.show()


def plot_results(cop_x: np.ndarray,
                 com_x: np.ndarray,
                 cop_y: np.ndarray,
                 com_y: np.ndarray,
                 zk_min_x: np.ndarray,
                 zk_max_x: np.ndarray,
                 zk_min_y: np.ndarray,
                 zk_max_y: np.ndarray,
                 theta_ref: np.ndarray,
                 foot_dimensions: List[float],
                 spacing: float
                 ) -> None:
    plt.plot(cop_x, label="cop")
    plt.plot(com_x, label="com")
    plt.plot(zk_min_x, linewidth=0.7)
    plt.plot(zk_max_x, linewidth=0.7)
    plt.title("x movement")
    # plt.ylim(0,2)
    plt.legend()
    plt.show()

    plt.plot(cop_y, label="cop")
    plt.plot(com_y, label="com")
    plt.plot(zk_min_y, linewidth=0.7)
    plt.plot(zk_max_y, linewidth=0.7)
    # plt.ylim((-0.8, 0.8))
    plt.title("y movement")
    plt.legend()
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(cop_x, cop_y, label="cop", color="green")
    ax.plot(com_x, com_y, label="com", color="red")
    # Plot footsteps
    plot_foot_steps(ax, (zk_min_x+zk_max_x)/2, (zk_min_y+zk_max_y)/2, theta_ref, foot_dimensions, spacing)
    # Display the plot
    plt.legend()
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    # ax.set_ylim((-0.03, 0.03))
    plt.title("Trajectory of robot")
    plt.show()


