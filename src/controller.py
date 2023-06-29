import numpy as np

from robot import Robot
from footstep_planner import FootstepPlanner
from utils import *
import visuals


class MPC:
    def __init__(self,
                 simulation_time: float,
                 prediction_time : float,
                 T_control: float,
                 T_pred: float,
                 robot: Robot,
                 footstep_planner: FootstepPlanner,
                 alpha: float,
                 beta: float,
                 gamma: float,
                 xk_init: (float, float, float),
                 yk_init: (float, float, float),
                 solver: str = "quadprog",
                 g : float = 9.81,
                 debug: bool = False,
                 ) -> None:
        self.simulation_time = simulation_time
        self.prediction_time = prediction_time
        self.T_control = T_control
        self.T_pred = T_pred
        self.robot = robot
        self.footstep_planner = footstep_planner
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.xk_init = xk_init
        self.yk_init = yk_init
        self.solver = solver
        self.g = g
        self.debug = debug

    def construct_objective(self,
                            T : float,
                            zk_ref_pred_x : np.ndarray,
                            zk_ref_pred_y : np.ndarray,
                            speed_ref_x_pred : np.ndarray,
                            speed_ref_y_pred : np.ndarray,
                            prev_x : np.ndarray,
                            prev_y : np.ndarray) -> (np.ndarray, np.ndarray):
        # Construct the objective function
        # Problem matrices
        N = int(self.prediction_time / self.T_pred)
        Pvu = p_v_u_matrix(T, N)
        Pvs = p_v_s_matrix(T, N)
        Pzu = p_z_u_matrix(T, self.robot.h, self.g, N)
        Pzs = p_z_s_matrix(T, self.robot.h, self.g, N)
        Qprime = self.beta * Pvu.T @ Pvu + self.alpha * np.eye(N) + self.gamma * Pzu.T @ Pzu
        Q = np.block([[Qprime, np.zeros(shape=(N, N))], [np.zeros(shape=(N, N)), Qprime]])

        p = np.hstack(
            (self.beta * Pvu.T @ (Pvs @ prev_x - speed_ref_x_pred) + self.gamma * Pzu.T @ (Pzs @ prev_x - zk_ref_pred_x),
             self.beta * Pvu.T @ (Pvs @ prev_y - speed_ref_y_pred) + self.gamma * Pzu.T @ (Pzs @ prev_y - zk_ref_pred_y)))

        return Q, p

    def construct_constraints(self,
                              T : float,
                              theta_ref_pred : np.ndarray,
                              zk_ref_pred_x : np.ndarray,
                              zk_ref_pred_y : np.ndarray,
                              prev_x : np.ndarray,
                              prev_y : np.ndarray) -> (np.ndarray, np.ndarray):
        # Construct the constraints
        N = int(self.prediction_time / self.T_pred)
        foot_dimensions = self.robot.foot_dimensions
        Pzu = p_z_u_matrix(T, self.robot.h, self.g, N)
        Pzs = p_z_s_matrix(T, self.robot.h, self.g, N)
        D = Dk_matrix(N, theta_ref_pred)
        b = np.array(
            [foot_dimensions[0] / 2, foot_dimensions[0] / 2, foot_dimensions[1] / 2, foot_dimensions[1] / 2] * N)
        G = D @ np.block([[Pzu, np.zeros(shape=(N, N))], [np.zeros(shape=(N, N)), Pzu]])
        h_cond = b + D @ np.hstack((zk_ref_pred_x - Pzs @ prev_x, zk_ref_pred_y - Pzs @ prev_y))
        return G, h_cond

    def run_MPC(self) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray):
        # Outputs
        com_x, com_y = [], []
        com_velocity_x, com_velocity_y = [], []
        com_acceleration_x, com_acceleration_y = [], []
        cop_x, cop_y = [], []
        prev_x, prev_y = self.xk_init, self.yk_init
        N = int(self.prediction_time / self.T_pred)

        # Run the simulation
        T = self.T_pred
        for i in range(int(self.simulation_time / self.T_control)):
            # Get the current prediction horizon
            curr_horizon_init, curr_horizon_end = i * self.prediction_time, (i+1)*self.prediction_time
            # TODO: Change the first T here
            zk_min_x, zk_max_x, zk_min_y, zk_max_y, theta_ref_pred = \
                self.footstep_planner.footsteps_to_array(curr_horizon_init, curr_horizon_end, self.T_pred)

            speed_ref_x_pred, speed_ref_y_pred = self.footstep_planner.speed_plan(curr_horizon_init,
                                                                                  curr_horizon_end, self.T_pred)

            assert(len(speed_ref_x_pred) == len(speed_ref_y_pred) == N)

            zk_ref_pred_x = (zk_min_x + zk_max_x) / 2
            zk_ref_pred_y = (zk_min_y + zk_max_y) / 2

            # Construct the objective function
            Q, p = self.construct_objective(T, zk_ref_pred_x, zk_ref_pred_y,
                                            speed_ref_x_pred, speed_ref_y_pred, prev_x, prev_y)

            G, h_cond = self.construct_constraints(T, theta_ref_pred, zk_ref_pred_x, zk_ref_pred_y, prev_x, prev_y)

            # Solve the QP
            jerk = solve_qp(P=Q, q=p, G=G, h=h_cond, solver=self.solver)

            if jerk is None:
                print(f"Cannot solve the QP at iteration {i}, most likely the value of xk diverges")
                return

            # Choosing the proper time step
            if i > 0:
                T -= (i % int(self.T_pred / self.T_control)) * self.T_control
            if T <= 0:
                T = self.T_pred

            # Compute the next state

            next_x, next_y = next_com(jerk=jerk[0], previous=prev_x, t_step=T), \
                             next_com(jerk=jerk[N], previous=prev_y, t_step=T)
            com_x.append(next_x[0])
            com_y.append(next_y[0])
            com_velocity_x.append(next_x[1])
            com_velocity_y.append(next_y[1])
            com_acceleration_x.append(next_x[2])
            com_acceleration_y.append(next_y[2])
            cop_x.append(np.array([1, 0, -self.robot.h / self.g]) @ next_x)
            cop_y.append(np.array([1, 0, -self.robot.h / self.g]) @ next_y)

            if self.debug:
                if i % 5 == 0:
                    visuals.plot_intermediate_states(i, prev_x, prev_y, self.prediction_time, self.T_pred, T, jerk,
                                                     self.robot.h, self.g, N, zk_ref_pred_x,
                                                     zk_ref_pred_y, theta_ref_pred, self.robot.foot_dimensions)

            # Update the status of the position
            prev_x, prev_y = next_x, next_y

        return cop_x, com_x, cop_y, com_y


