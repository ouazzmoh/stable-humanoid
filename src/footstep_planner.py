import numpy as np
from typing import List, Tuple

from step import Step
from robot import Robot
import scenarios

class FootstepPlanner:
    def __init__(self,
                 robot: Robot,
                 simulation_time: float,
                 duration_double_init: float,
                 duration_step: float,
                 trajectory_type: str,
                 average_speed: (float, float) = (0, 0),
                 stop_at: (float, float) = (0, 0)  # "forward", "upwards", "upwards_turning"
                 ) -> None:
        self.robot = robot
        self.simulation_time = simulation_time
        self.duration_double_init = duration_double_init
        self.duration_step = duration_step
        self.trajectory_type = trajectory_type

        # Construct the trajectory for the robot
        footsteps_x, footsteps_y = None, None
        if self.trajectory_type == "forward":
            footsteps_x = scenarios.construct_zmin_zmax_forward(self.simulation_time,
                                                                self.duration_double_init,
                                                                self.duration_step,
                                                                self.robot.foot_dimensions[0],
                                                                self.robot.spacing_x)
            footsteps_y = scenarios.construct_zmin_zmax_alt(self.simulation_time,
                                                            self.duration_double_init,
                                                            self.duration_step,
                                                            self.robot.foot_dimensions[1],
                                                            self.robot.spacing_y)

        elif self.trajectory_type == "upwards":
            footsteps_x = scenarios.construct_zmin_zmax_forward(self.simulation_time,
                                                                self.duration_double_init,
                                                                self.duration_step,
                                                                self.robot.foot_dimensions[0],
                                                                self.robot.spacing_x)
            footsteps_y = scenarios.construct_zmin_zmax_alt_forward(self.simulation_time,
                                                                    self.duration_double_init,
                                                                    self.duration_step,
                                                                    self.robot.foot_dimensions[1],
                                                                    self.robot.spacing_y,
                                                                    shift=0.35)
            # Changing the orientation for the feet
            assert (len(footsteps_x) == len(footsteps_y))
            for i in range(len(footsteps_x)):
                footsteps_x[i].orientation = np.pi / 4
                footsteps_y[i].orientation = np.pi / 4

        elif self.trajectory_type == "upwards_turning":
            footsteps_x = scenarios.construct_zmin_zmax_forward_backwards(self.simulation_time,
                                                                          self.duration_double_init,
                                                                          self.duration_step,
                                                                          self.robot.foot_dimensions[0],
                                                                          self.robot.spacing_x)
            footsteps_y = scenarios.construct_zmin_zmax_alt_forward(self.simulation_time,
                                                                    self.duration_double_init,
                                                                    self.duration_step,
                                                                    self.robot.foot_dimensions[1],
                                                                    self.robot.spacing_y,
                                                                    shift=0.35)
            footsteps_y.append(footsteps_y[-1])
            assert (len(footsteps_x) == len(footsteps_y))
            for i in range(int(len(footsteps_x)*0.5)):
                footsteps_x[i].orientation = np.pi / 4
                footsteps_y[i].orientation = np.pi / 4
            for i in range(int(len(footsteps_x) * 0.5), len(footsteps_x)):
                footsteps_x[i].orientation = np.pi / 4 + np.pi/2
                footsteps_y[i].orientation = np.pi / 4 + np.pi/2
        elif self.trajectory_type == "interactive":
            footsteps_x, footsteps_y = scenarios.construct_zmin_zmax_interactive((0, 4), (-1.5, 1.5),
                                                                                 self.simulation_time,
                                                                                 self.duration_double_init,
                                                                                 self.duration_step,
                                                                                 self.robot.foot_dimensions,
                                                                                 (self.robot.spacing_x,
                                                                                  self.robot.spacing_y))
        else:
            raise ValueError("Invalid trajectory type: the available scenarios are 'forward', 'upwards', "
                             "'upwards_turning'")

        self.footsteps_x, self.footsteps_y = footsteps_x, footsteps_y
        self.average_speed = average_speed

        # The reference speed of the robot
        # the speed attribute is in the form [(start_time, end_time, speed), ...]
        if average_speed[0] > 0:
            self.speed_x = [(0, duration_double_init, 0), (duration_double_init, stop_at[0], average_speed[0]),
                            (stop_at[0], simulation_time, 0)]
        else:
            self.speed_x = [(0, simulation_time, 0)]
        if average_speed[1] > 0:
            self.speed_y = [(0, duration_double_init, 0), (duration_double_init, stop_at[1], average_speed[1]),
                            (stop_at[1], simulation_time, 0)]
        else:
            self.speed_y = [(0, simulation_time, 0)]



    def extend_time(self, by_time: float) -> None:
        self.footsteps_x[-1].end_time += by_time
        self.footsteps_y[-1].end_time += by_time


    def get_footsteps(self,
                      from_time: float,
                      to_time: float) -> List[Tuple[float, float, str, float]]:
        """
        Get the footsteps from the given time interval
        Args:
            from_time:
            to_time:

        Returns:

        """
        assert(len(self.footsteps_x) == len(self.footsteps_y))
        start_index, end_index = 0, len(self.footsteps_x)
        steps = []

        for i in range(len(self.footsteps_x)):
            if self.footsteps_x[i].start_time <= from_time <= self.footsteps_x[i].end_time:
                start_index = i
            if self.footsteps_x[i].start_time <= to_time <= self.footsteps_x[i].end_time:
                end_index = i

        for k in range(start_index, end_index+1):
            step_k = (((self.footsteps_x[k].z_max + self.footsteps_x[k].z_min)/2),
                      ((self.footsteps_y[k].z_max + self.footsteps_y[k].z_min) / 2),
                      self.footsteps_x[k].which_foot,
                      self.footsteps_x[k].orientation)
            steps.append(step_k)
        return steps






    def footsteps_to_array(self,
                           from_time: float,
                           to_time: float,
                           T : float) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray):
        """
        Get the footsteps from the given time interval as a numpy array sampled at the given period T
        Args:
            from_time:
            to_time:
            T: sampling period

        Returns:

        """

        assert(len(self.footsteps_x) == len(self.footsteps_y))
        # Detecting the index of the start and finish
        start_index, end_index = None, None
        zk_min_x, zk_max_x = None, None
        zk_min_y, zk_max_y = None, None
        start_step_percentage, end_step_percentage = None, None
        theta = None

        for i in range(len(self.footsteps_x)):
            if self.footsteps_x[i].start_time <= from_time and self.footsteps_x[i].end_time >= from_time:
                start_index = i
                start_step_percentage = (self.footsteps_x[i].end_time - from_time) / (self.footsteps_x[i].end_time -
                                                                                      self.footsteps_x[i].start_time)
            if self.footsteps_x[i].start_time <= to_time and self.footsteps_x[i].end_time >= to_time:
                end_index = i
                end_step_percentage = (to_time - self.footsteps_x[i].start_time) / (self.footsteps_x[i].end_time -
                                                                                    self.footsteps_x[i].start_time)

        duration = self.footsteps_x[0].end_time - self.footsteps_x[0].start_time  # Duration of the step is constant in our implementation

        if start_index is None:
            zk_min_x = [self.footsteps_x[-1].z_min] * round(duration / T)
            zk_max_x = [self.footsteps_x[-1].z_max] * round(duration / T)
            zk_min_y = [self.footsteps_y[-1].z_min] * round(duration / T)
            zk_max_y = [self.footsteps_y[-1].z_max] * round(duration / T)
            theta = [self.footsteps_x[-1].orientation] * round(duration / T)
        elif end_index is None or end_index == start_index:
            zk_min_x = [self.footsteps_x[start_index].z_min] * round(duration/T)
            zk_max_x = [self.footsteps_x[start_index].z_max] * round(duration/T)
            zk_min_y = [self.footsteps_y[start_index].z_min] * round(duration/T)
            zk_max_y = [self.footsteps_y[start_index].z_max] * round(duration/T)
            theta = [self.footsteps_x[start_index].orientation] * round(duration/T)
        else:
            for k in range(start_index, end_index+1):
                if k == start_index:
                    zk_min_x = [self.footsteps_x[k].z_min] * round((start_step_percentage*duration)/T)
                    zk_max_x = [self.footsteps_x[k].z_max] * round((start_step_percentage*duration)/T)
                    zk_min_y = [self.footsteps_y[k].z_min] * round((start_step_percentage*duration)/T)
                    zk_max_y = [self.footsteps_y[k].z_max] * round((start_step_percentage*duration)/T)
                    theta = [self.footsteps_x[k].orientation] * round((start_step_percentage*duration)/T)
                elif k == end_index:
                    zk_min_x += [self.footsteps_x[k].z_min] * round((end_step_percentage*duration)/T)
                    zk_max_x += [self.footsteps_x[k].z_max] * round((end_step_percentage*duration)/T)
                    zk_min_y += [self.footsteps_y[k].z_min] * round((end_step_percentage*duration)/T)
                    zk_max_y += [self.footsteps_y[k].z_max] * round((end_step_percentage*duration)/T)
                    theta += [self.footsteps_x[k].orientation] * round((end_step_percentage*duration)/T)
                else:
                    zk_min_x += [self.footsteps_x[k].z_min] * round(duration/T)
                    zk_max_x += [self.footsteps_x[k].z_max] * round(duration/T)
                    zk_min_y += [self.footsteps_y[k].z_min] * round(duration/T)
                    zk_max_y += [self.footsteps_y[k].z_max] * round(duration/T)
                    theta += [self.footsteps_x[k].orientation] * round(duration/T)
        # padding
        zk_min_x += [zk_min_x[-1]]*(round((to_time-from_time)/T) - len(zk_min_x))
        zk_min_x = zk_min_x[:round((to_time-from_time)/T)]
        zk_max_x += [zk_max_x[-1]]*(round((to_time-from_time)/T) - len(zk_max_x))
        zk_max_x = zk_max_x[:round((to_time-from_time)/T)]
        zk_min_y += [zk_min_y[-1]]*(round((to_time-from_time)/T) - len(zk_min_y))
        zk_min_y = zk_min_y[:round((to_time-from_time)/T)]
        zk_max_y += [zk_max_y[-1]]*(round((to_time-from_time)/T) - len(zk_max_y))
        zk_max_y = zk_max_y[:round((to_time-from_time)/T)]
        theta += [theta[-1]]*(round((to_time-from_time)/T) - len(theta))
        theta = theta[:round((to_time-from_time)/T)]

        return np.array(zk_min_x), np.array(zk_max_x), np.array(zk_min_y), np.array(zk_max_y), np.array(theta)

    @staticmethod
    def _speed_to_array(speed: List[Tuple[float, float, float]],
                       from_time: float,
                       to_time: float,
                       T : float) -> np.ndarray:
        # Detecting the index of the start and finish
        start_index, end_index = None, None
        start_step_percentage, end_step_percentage = None, None
        speed_ret = []
        for i in range(len(speed)):
            start, end, val = speed[i][0], speed[i][1], speed[i][2]
            if start <= from_time <= end:
                start_index = i
                start_step_percentage = (end - from_time) / (end - start)
            if start <= to_time <= end:
                end_index = i
                end_step_percentage = (to_time - start) / (end - start)

        if start_index is None:
            return np.array([speed[-1][2]]*round((to_time-from_time)/T))
        elif end_index is None or start_index == end_index:
            return np.array([speed[start_index][2]]*round((to_time-from_time)/T))
        else:
            for k in range(start_index, end_index+1):
                if k == start_index:
                    speed_ret = [speed[k][2]] * round((start_step_percentage*(speed[k][1] - speed[k][0]))/T)
                elif k == end_index:
                    speed_ret += [speed[k][2]] * round((end_step_percentage*(speed[k][1] - speed[k][0]))/T)
                else:
                    speed_ret += [speed[k][2]] * round((speed[k][1] - speed[k][0])/T)

            # padding
            speed_ret += [speed_ret[-1]]*(round((to_time-from_time)/T) - len(speed_ret))
        return np.array(speed_ret[0:round((to_time-from_time)/T)])

    def speed_plan(self,
                   from_time: float,
                   to_time: float,
                   T : float ) -> (np.ndarray, np.ndarray):
        speed_x, speed_y = None, None
        if self.average_speed[0] == 0:
            speed_x = np.zeros(round((to_time-from_time)/T))
        else:
            speed_x = self._speed_to_array(self.speed_x, from_time, to_time, T)

        if self.average_speed[1] == 0:
            speed_y = np.zeros(round((to_time-from_time)/T))
        else:
            speed_y = self._speed_to_array(self.speed_y, from_time, to_time, T)
        return speed_x, speed_y





