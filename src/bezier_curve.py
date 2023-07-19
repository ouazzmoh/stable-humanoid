from utils import *
import matplotlib.pyplot as plt
import numpy as np

class BezierCurve:
    """ 
    Class for Bezier curves. 
    """
    def __init__(self, control_points, nb_ti=100):
        self.control_points = control_points
        self.nb_ti = nb_ti
        self.tx, self.ty, self.tz = self.__get_trajectory(
            control_points[:, 0], control_points[:, 1], control_points[:, 2]
        )
        self.plot_2D = np.all(self.tz == 0)

    def __get_trajectory(self, x_control_points, y_control_points, z_control_points):
        """
        Compute the trajectory of the Bezier curve.

        Args:
            x_control_points : x coordinates of the control points.
            y_control_points : y coordinates of the control points.
            z_control_points : z coordinates of the control points.

        Returns:
            _type_: _description_
        """
        degree = len(x_control_points) - 1
        t = np.linspace(0, 1, self.nb_ti)
        tx = np.zeros(self.nb_ti)
        ty = np.zeros(self.nb_ti)
        tz = np.zeros(self.nb_ti)
        for k in range(degree + 1):
            poly = Bernstein(degree, k, t)
            tx = tx + x_control_points[k] * poly
            ty = ty + y_control_points[k] * poly
            tz = tz + z_control_points[k] * poly
        return tx, ty, tz

    def plot_curve(self, ax=plt.figure(), color="r", title=None, plot_points=True):
        """
        Plot the Bezier curve.

        Args:
            ax: figure to plot on. Defaults to plt.figure().
            color: color of the curve. Defaults to "r".
            title: title of the plot. Defaults to None.
            plot_points: whether to plot the control points. Defaults to True.
        """
        if not self.plot_2D:
            ax = ax.add_subplot(projection="3d")
            ax.plot(self.tx, self.ty, self.tz, color)
            if plot_points:
                ax.scatter(
                    self.control_points[:, 0],
                    self.control_points[:, 1],
                    self.control_points[:, 2],
                    c="g",
                )
        else:
            ax.plot(self.tx, self.ty, color)
            if plot_points:
                ax.scatter(self.control_points[:, 0], self.control_points[:, 1], c="g")
        if title:
            plt.title(title)
        plt.show()
    
    def get_position_at(self, t: float):
        """
        Get the position of the Bezier curve at time t.
        
        Args:
            t: time at which to get the position.

        Returns:
            numpy array: position of the Bezier curve at time t.
        """
        assert 0 <= t <= 1, " t should be between 0 and 1"
        index = int(t * self.nb_ti) - 1 if t > 0 else 0
        return np.array([self.tx[index], self.ty[index], self.tz[index]])