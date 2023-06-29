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
                 average_speed: (float, float) = (0, 1),
                 stop_at: (float, float) = (0, 1)  # "forward", "upwards", "upwards_turning"
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
            footsteps_y = scenarios.construct_zmin_zmax(self.simulation_time,
                                                        self.duration_double_init,
                                                        self.duration_step,
                                                        self.robot.foot_dimensions[1],
                                                        self.robot.spacing_y)

        self.footsteps_x, self.footsteps_y = footsteps_x, footsteps_y

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

    def footsteps_to_array(self,
                           from_time: float,
                           to_time: float,
                           T : float) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray):

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
        zk_max_x += [zk_max_x[-1]]*(round((to_time-from_time)/T) - len(zk_max_x))
        zk_min_y += [zk_min_y[-1]]*(round((to_time-from_time)/T) - len(zk_min_y))
        zk_max_y += [zk_max_y[-1]]*(round((to_time-from_time)/T) - len(zk_max_y))
        theta += [theta[-1]]*(round((to_time-from_time)/T) - len(theta))

        return np.array(zk_min_x), np.array(zk_max_x), np.array(zk_min_y), np.array(zk_max_y), np.array(theta)

    @staticmethod
    def _speed_to_array(self,
                       speed: List[Tuple[float, float, float]],
                       from_time: float,
                       to_time: float,
                       T : float) -> np.ndarray:
        # Detecting the index of the start and finish
        start_index, end_index = None, None
        start_step_percentage, end_step_percentage = None, None
        speed_ret = []
        for i in range(len(speed)):
            start, end, val = speed[i][0], speed[i][1], speed[i][2]
            if start <= from_time and end >= from_time:
                start_index = i
                start_step_percentage = (end - from_time) / (end - start)
            if start <= to_time and end >= to_time:
                end_index = i
                end_step_percentage = (to_time - start) / (end - start)

        for k in range(start_index, end_index+1):
            if k == start_index:
                speed_ret = [speed[k][2]] * round((start_step_percentage*(speed[k][1] - speed[k][0]))/T)
            elif k == end_index:
                speed_ret += [speed[k][2]] * round((end_step_percentage*(speed[k][1] - speed[k][0]))/T)
            else:
                speed_ret += [speed[k][2]] * round((speed[k][1] - speed[k][0])/T)

        # padding
        speed_ret += [speed_ret[-1]]*(round((to_time-from_time)/T) - len(speed_ret))
        return np.array(speed_ret)

    def speed_plan(self,
                   from_time: float,
                   to_time: float,
                   T : float ) -> (np.ndarray, np.ndarray):
        speed_x = self._speed_to_array(self.speed_x, from_time, to_time, T)
        speed_y = self._speed_to_array(self.speed_y, from_time, to_time, T)
        return speed_x, speed_y





