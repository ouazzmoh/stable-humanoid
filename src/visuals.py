import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


def plot_foot_steps(ax, zk_ref_x, zk_ref_y, foot_dimensions) -> None:
    """
    Plot the footsteps of the robot
    :param ax:  matplotlib ax returned by subplots
    :param zk_ref_x: array of cop center references for x
    :param zk_ref_y: array of cop center references for y
    :param foot_dimensions:  (length (x), width (y))
    :return:
    """
    # Combine the two arrays to create an array of points
    points = np.array([zk_ref_x, zk_ref_y]).T
    # Get unique rows,
    _, idx = np.unique(points, axis=0, return_index=True)
    unique_points = points[np.sort(idx)]
    # Now your x and y coordinates are the first and second columns of unique_points
    steps_x = unique_points[:, 0]
    steps_y = unique_points[:, 1]
    plt.plot(steps_x, steps_y, 'go', label="ref_center")
    # Single supports

    for x, y in zip(steps_x[1:-1], steps_y[1:-1]):
        # Create a rectangle
        # TODO: Now we rescale the width by 20% -> make this general
        rect = patches.Rectangle((x - foot_dimensions[0] / 2, y - 0.2 * foot_dimensions[1] / 2),
                                 foot_dimensions[0], 0.2 * foot_dimensions[1],
                                 linewidth=0.7, edgecolor='grey', facecolor='none')
        # Add the rectangle to the plot
        ax.add_patch(rect)
    # Double supports
    for x, y in zip([steps_x[0], steps_x[-1]], [steps_y[0], steps_y[-1]]):
        rect1 = patches.Rectangle((x - foot_dimensions[0] / 2, y - 1.2 * foot_dimensions[1] / 2),
                                  foot_dimensions[0], 0.2 * foot_dimensions[1],
                                  linewidth=0.7, edgecolor='grey', facecolor='none')
        rect2 = patches.Rectangle((x - foot_dimensions[0] / 2, y + 0.8 * foot_dimensions[1] / 2),
                                  foot_dimensions[0], 0.2 * foot_dimensions[1],
                                  linewidth=0.7, edgecolor='grey', facecolor='none')
        ax.add_patch(rect1)
        ax.add_patch(rect2)