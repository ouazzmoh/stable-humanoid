from typing import Tuple, List
from robot import Robot
from perturbation import Perturbation
from footstep_planner import FootstepPlanner
from utils import *
import visuals



class MPC:
    """
    This class represents the MPC controller, which is used to generate stable walking for the robot.

    Attributes:
        simulation_time: total simulation time (s)
        prediction_time: time for the prediction horizon (s)
        T_control: sampling time for the control (s)
        T_pred: sampling time for the prediction (s)
        robot: robot object
        footstep_planner: footstep planner object, which is used to generate the reference trajectories
        alpha: weight for the jerk
        beta: weight for the velocity
        gamma: weight for the reference trajectory
        xk_init: initial state of the robot in the x direction (position, velocity, acceleration)
        yk_init: initial state of the robot in the y direction (position, velocity, acceleration)
        name (str): name of the controller
        write_hdf5 (bool): whether to write the results to a hdf5 file
        solver (str): solver to use for the QP problem
        g (float): gravity constant
        debug (bool): whether to plot intermediate trajectories for the robot
        perturbations (List[Perturbation]): list of perturbations to apply to the robot
    """
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
                 name: str = "Walking_MPC",
                 write_hdf5: bool = False,
                 solver: str = "quadprog",
                 g: float = 9.81,
                 debug: bool = False,
                 perturbations: List[Perturbation] = None,
                 ) -> None:
        self.simulation_time = simulation_time
        self.prediction_time = prediction_time
        self.T_control = T_control
        self.T_pred = T_pred
        self.robot = robot



        # Padding the footsteps, to consider the last step's end time until the last prediction time
        footstep_planner.extend_time(prediction_time)
        self.footstep_planner = footstep_planner



        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.xk_init = xk_init
        self.yk_init = yk_init
        self.name = name
        self.write_hdf5 = write_hdf5
        self.solver = solver
        self.g = g
        self.debug = debug
        self.perturbations = perturbations

    def construct_objective(self,
                            T : float,
                            zk_ref_pred_x : np.ndarray,
                            zk_ref_pred_y : np.ndarray,
                            speed_ref_x_pred : np.ndarray,
                            speed_ref_y_pred : np.ndarray,
                            prev_x : np.ndarray,
                            prev_y : np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Represents the objective function of the QP problem to minimize,
                in :math:`[\mathrm{cost}] / [\mathrm{dimensionless}]`. The function
                is composed of three parts:

                1. :math:`\frac{\alpha}{2} \|Xdddot_k_\|^2` This term corresponds to
                the error related to the jerk

                2. :math:`\frac{\beta}{2} \|Xdot_{k+1} - Xdot_{k+1}^{ref}\|^2` This
                term corresponds to the difference between the actual and reference
                speed.

                3. :math:`\frac{\gamma}{2} \|Z_{k}^{x} - Z_{k}^{x_ref}\|^2` This term
                corresponds to the error in the position of the center of pressure (CoP)
                in the x direction.

                In addition to the same terms for the y direction.

        Args:
            T: Sampling period
            zk_ref_pred_x: cop reference in the x direction
            zk_ref_pred_y: cop reference in the y direction
            speed_ref_x_pred: speed reference in the x direction
            speed_ref_y_pred: speed reference in the y direction
            prev_x: last state before the resolution of the QP in the x direction :
            (com_position, com_speed, com_acceleration)
            prev_y: last state before the resolution of the QP in the y direction

        Returns:
            Q: Quadratic term of the objective function
            p: Linear term of the objective function

        """
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
                              prev_y : np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Represents the constraints of the QP problem to minimize:
        Assert that the cop stays with the bounds of the support polygon by asserting the condition :
        Args:
            T: sampling period
            theta_ref_pred: reference angle of the robot
            zk_ref_pred_x: reference cop in the x direction
            zk_ref_pred_y: reference cop in the y direction
            prev_x: last state before the resolution of the QP in the x direction :
            prev_y: last state before the resolution of the QP in the y direction

        Returns:

        """

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

    def get_prediction_horizon_data(self,
                                    i: int,
                                    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray,
                                               List[Tuple]]:
        """
        Get the prediction horizon data for the current iteration
        Args:
            i: current iteration
        Returns:
            zk_ref_pred_x: reference x position of the footsteps in the prediction horizon
            zk_ref_pred_y: reference y position of the footsteps in the prediction horizon
            speed_ref_x_pred: reference x speed of the footsteps in the prediction horizon
            speed_ref_y_pred: reference y speed of the footsteps in the prediction horizon
            theta_ref_pred: reference orientation of the footsteps in the prediction horizon
        """
        curr_horizon_init, curr_horizon_end = i * self.T_control, i * self.T_control + self.prediction_time
        zk_min_x, zk_max_x, zk_min_y, zk_max_y, theta_ref_pred = \
            self.footstep_planner.footsteps_to_array(curr_horizon_init, curr_horizon_end, self.T_pred)
        speed_ref_x_pred, speed_ref_y_pred = self.footstep_planner.speed_plan(curr_horizon_init,
                                                                              curr_horizon_end, self.T_pred)
        zk_ref_pred_x = (zk_min_x + zk_max_x) / 2
        zk_ref_pred_y = (zk_min_y + zk_max_y) / 2

        steps = self.footstep_planner.get_footsteps(curr_horizon_init, curr_horizon_end)

        return zk_ref_pred_x, zk_ref_pred_y, speed_ref_x_pred, speed_ref_y_pred, theta_ref_pred, steps

    def MPC_iteration(self,
                      i: int,
                      N: int,
                      T: float,
                      file=None) -> None:
        """
        Perform one iteration of the MPC, i.e. solve the QP of stable walking
        The function doesn't return anything, but it updates the positional arguments of the robot
        (attribute of controller)
        Args:
            i: current iteration of the MPC loop
            N: Number of sample in the prediction horizon i.e. QP problem size
            T: The proper step of integration: used for intermediate visualizations
            file: The file to store the QP problem if needed
        Raises:
            ValueError: If the robot com position is not initialized
            ValueError: If the QP problem is not feasible
        Returns:
            None

        """
        if self.robot.com_position is None:
            raise ValueError("The robot com position is not initialized")

        # Current state of the robot
        curr_xk = np.array([self.robot.com_position[0], self.robot.com_velocity[0], self.robot.com_acceleration[0]])
        curr_yk = np.array([self.robot.com_position[1], self.robot.com_velocity[1], self.robot.com_acceleration[1]])
        # Get the current prediction horizon
        zk_ref_pred_x, zk_ref_pred_y, speed_ref_x_pred, speed_ref_y_pred, theta_ref_pred, steps = \
            self.get_prediction_horizon_data(i)



        # TODO : Remove this assertion
        assert(len(zk_ref_pred_x) == len(zk_ref_pred_y) == len(speed_ref_y_pred) == len(speed_ref_x_pred)
               == len(theta_ref_pred) == N)

        # Construct the objective function
        Q, p = self.construct_objective(self.T_pred, zk_ref_pred_x, zk_ref_pred_y,
                                        speed_ref_x_pred, speed_ref_y_pred, curr_xk, curr_yk)

        G, h_cond = self.construct_constraints(self.T_pred, theta_ref_pred, zk_ref_pred_x, zk_ref_pred_y,
                                               curr_xk, curr_yk)
        # Solve the QP
        solution = solve_qp(P=Q, q=p, G=G, h=h_cond, solver=self.solver)

        if solution is None:
            raise ValueError(f"Cannot solve the QP at iteration {i}")

        if file and self.write_hdf5:
            store_qp_in_file(file, self.T_control * i, f"mpc_qp_{i:04}", "mpc", P=Q, q=p, G=G, h=h_cond)
            # TODO: Remove the assertions below
            # assert np.all(Q == retrieve_problem_data_from_file(file, i)["P"])
            # assert np.all(p == retrieve_problem_data_from_file(file, i)["q"])
            # assert np.all(G == retrieve_problem_data_from_file(file, i)["G"])
            # assert np.all(h_cond == retrieve_problem_data_from_file(file, i)["h"])

        if self.debug:
            num_frames = 10  # Number of frames to plot
            if i % (int(self.simulation_time / self.T_control) // num_frames) == 0:
                visuals.plot_intermediate_states(i, curr_xk, curr_yk, self.prediction_time, self.T_pred, T, solution,
                                                 self.robot.h, self.g, N, zk_ref_pred_x,
                                                 zk_ref_pred_y, theta_ref_pred, self.robot.foot_dimensions)
            T -= self.T_control
            if T <= 0:
                T = self.T_pred
        # Compute the next state
        # We integrate using the control time step
        next_x, next_y = next_com(jerk=solution[0], previous=curr_xk, t_step=self.T_control), \
                         next_com(jerk=solution[N], previous=curr_yk, t_step=self.T_control)
        # Add the perturbations is they exist
        if self.perturbations:
            for perturb in self.perturbations:
                if abs(perturb.time - i * self.T_control) <= self.T_control:
                    next_x[2] += perturb.value_x
                    next_y[2] += perturb.value_y
        # Update the status of the position
        self.robot.set_positional_attributes(next_x, next_y, steps, self.g)


    def run_MPC(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Run the full iterations of the MPC
        Returns:
            com_x: The x position of the center of mass
            com_y: The y position of the center of mass
            com_velocity_x: The x velocity of the center of mass
            com_velocity_y: The y velocity of the center of mass
            com_acceleration_x: The x acceleration of the center of mass
            com_acceleration_y: The y acceleration of the center of mass
            cop_x: The x position of the center of pressure
            cop_y: The y position of the center of pressure
        """
        # Outputs
        com_x, com_y = [], []
        com_velocity_x, com_velocity_y = [], []
        com_acceleration_x, com_acceleration_y = [], []
        cop_x, cop_y = [], []
        file = None
        # Parameters
        N = int(self.prediction_time / self.T_pred)
        T = self.T_pred  # The proper step of integration: used for intermediate visualizations

        # Initialize state of the robot
        self.robot.initialize_position(self.xk_init, self.yk_init, self.g)

        # Run the simulation
        if self.write_hdf5:
            # The storing process currently supports up to 4 digits 0 < i < 9999
            file_path = get_file_path(self.name)
            file = h5.File(file_path, "a")

        for i in range(int(self.simulation_time / self.T_control)):
            # Solve one iteration of the MPC and update the state of the robot
            self.MPC_iteration(i, N, T, file)

            # Store the results
            com_x.append(self.robot.com_position[0])
            com_y.append(self.robot.com_position[1])
            com_velocity_x.append(self.robot.com_velocity[0])
            com_velocity_y.append(self.robot.com_velocity[1])
            com_acceleration_x.append(self.robot.com_acceleration[0])
            com_acceleration_y.append(self.robot.com_acceleration[1])
            cop_x.append(self.robot.cop_position[0])
            cop_y.append(self.robot.cop_position[1])


        if self.write_hdf5 : file.close()

        return np.array(cop_x), np.array(com_x), np.array(cop_y), np.array(com_y)


