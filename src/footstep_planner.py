import numpy as np
from typing import List

from step import Step
from robot import Robot
import scenarios



class FootstepPlanner:
    def __init__(self,
                 robot: Robot,
                 simulation_time: float,
                 prediction_time: float,
                 duration_double_init: float,
                 duration_step: float,
                 trajectory_type: str,
                 average_speed: (float, float) = (0, 1),
                 stop_at: (float, float) = (0, 1)  # "forward", "upwards", "upwards_turning"
                 ) -> None:
        self.robot = robot
        self.simulation_time = simulation_time
        self.prediction_time = prediction_time
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
        self.speed_x = [(0, duration_double_init, 0), (duration_double_init, stop_at[0], average_speed[0]),
                        (stop_at[0], simulation_time, 0)]
        self.speed_y = [(0, duration_double_init, 0), (duration_double_init, stop_at[1], average_speed[1]),
                        (stop_at[1], simulation_time, 0)]

    def footsteps_to_array(self,
                           from_time: float,
                           to_time: float,
                           T : float) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray,
                                          np.ndarray, np.ndarray):

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

    def speed_to_array(self,
                       from_time: float,
                       to_time: float,
                       T : float) -> (np.ndarray, np.ndarray):
        # Detecting the index of the start and finish
        start_index, end_index = None, None
        start_step_percentage, end_step_percentage = None, None
        speed_x, speed_y = None, None

        for i in range(len(self.speed_x)):
            if self.speed_x[i][0] <= from_time and self.speed_x[i][1] >= from_time:
                start_index = i
                start_step_percentage = (self.speed_x[i][1] - from_time) / (self.speed_x[i][1] - self.speed_x[i][0])
            if self.speed_x[i][0] <= to_time and self.speed_x[i][1] >= to_time:
                end_index = i
                end_step_percentage = (to_time - self.speed_x[i][0]) / (self.speed_x[i][1] - self.speed_x[i][0])

        for k in range(start_index, end_index+1):
            if k == start_index:
                speed_x = [self.speed_x[k][2]] * round((start_step_percentage*(self.speed_x[k][1] - self.speed_x[k][0]))/T)
                speed_y = [self.speed_y[k][2]] * round((start_step_percentage*(self.speed_y[k][1] - self.speed_y[k][0]))/T)
            elif k == end_index:
                speed_x += [self.speed_x[k][2]] * round((end_step_percentage*(self.speed_x[k][1] - self.speed_x[k][0]))/T)
                speed_y += [self.speed_y[k][2]] * round((end_step_percentage*(self.speed_y[k][1] - self.speed_y[k][0]))/T)
            else:
                speed_x += [self.speed_x[k][2]] * round((self.speed_x[k][1] - self.speed_x[k][0])/T)
                speed_y += [self.speed_y[k][2]] * round((self.speed_y[k][1] - self.speed_y[k][0])/T)
        # padding
        speed_x += [speed_x[-1]]*(round((to_time-from_time)/T) - len(speed_x))
        speed_y += [speed_y[-1]]*(round((to_time-from_time)/T) - len(speed_y))
        return np.array(speed_x), np.array(speed_y)





