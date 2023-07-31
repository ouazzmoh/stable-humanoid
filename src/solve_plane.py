from qpsolvers import solve_qp

from robot_arm import RobotArm
from person import Person
import numpy as np


class SolvePlane:

    def __init__(self,
                 robot_arm: RobotArm,
                 person: Person,
                 simulation_time: float,
                 beta : float,
                 alpha: float,
                 epsilon: float,
                 solver: str = "quadprog"):
        self.robot_arm = robot_arm
        self.person = person
        self.simulation_time = simulation_time
        self.beta = beta
        self.alpha = alpha
        self.epsilon = epsilon
        self.solver = solver
        # TODO: Initialize it like this for now : Starting from the first point of the robot arm
        self.ak_p = (1, 0, 0)
        self.bk_p = 1.5

    def construct_objective_function(self,
                                     ak_p : np.ndarray,
                                     bk_p : float):

        mat = np.array([self.beta, 0, 0, self.alpha]).reshape((2, 2))
        Q = np.block([[self.beta * np.eye(3), np.zeros((3, 2))],
                      [np.zeros((2, 3)), mat]])

        p = np.array([-self.beta * ak_p[0], -self.beta * ak_p[1], -self.beta * ak_p[2], -self.alpha * bk_p, -1])

        return Q, p

    def construct_constraint_matrix(self,
                                    k : int,
                                    ak_p : np.ndarray):
        lines_G = []
        l1 = [*self.person.vertices[k], -1, 0]
        l2 = [*self.person.vertices[k+1], -1, 0]
        l3 = [*self.robot_arm.vertices[k], 1, 1]
        l4 = [*self.robot_arm.vertices[k+1], 1, 1]
        l5 = [*ak_p, 0, 0]
        l6 = [*ak_p, 0, 0]
        for i in range(3):
            l3[i] = -l3[i]
            l4[i] = -l4[i]
            l6[i] = -l6[i]
        lines_G.append(l1)
        lines_G.append(l2)
        lines_G.append(l3)
        lines_G.append(l4)
        lines_G.append(l5)
        lines_G.append(l6)

        G = np.array(lines_G)

        h = np.zeros(len(lines_G)-2)
        h = np.append(h, [1, self.epsilon - 1])

        lb = np.array([-1, -1, -1, -np.inf, -np.inf])
        ub = np.array([1, 1, 1, np.inf, np.inf])

        return G, h, lb, ub

    def run_iteration(self,
                      k : int):

        Q, p = self.construct_objective_function(self.ak_p, self.bk_p)

        G, h_cond, lb, ub = self.construct_constraint_matrix(k, self.ak_p)

        solution = solve_qp(P=Q, q=p, G=G, h=h_cond, lb=lb, ub=ub, solver=self.solver)

        if solution is None:
            # TODO: Treat this case properly
            raise ValueError(f"Cannot find the plane for iteration")

        curr_ak = solution[0:3]
        curr_bk = solution[3]
        curr_d = solution[4]

        self.ak_p = curr_ak
        self.bk_p = curr_bk

        return curr_ak, curr_bk, curr_d














