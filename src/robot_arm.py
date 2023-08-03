from typing import List, Tuple
import numpy as np


class RobotArm:
    def __init__(self,
                 init_q: List[float] = None,
                 init_r0: Tuple[float, float, float] = None,
                 L0 : float = None,
                 L1 : float = None,
                 vertices: List[List[Tuple[float, float, float]]] = None):
        self.init_q = init_q
        self.L0 = L0
        self.L1 = L1
        r0 = init_r0
        r1 = (r0[0] + L0 * np.cos(init_q[0]), r0[1] + L0 * np.sin(init_q[0]), r0[2])
        r2 = (r1[0] + L1 * np.cos(init_q[0] + init_q[1]), r1[1] + L1 * np.sin(init_q[0] + init_q[1]), r1[2])
        # If we pass exactly the vertices, we don't calculate them
        if vertices is None:
            vertices = [[r0], [r1], [r2]]
        else:
            self.vertices = vertices


    def plan_trajectory(self,
                        simulation_time: float,
                        T_control: float,
                        translation_step: float = 0.3) -> None:

        # Translate with
        for i in range(1, int(simulation_time / T_control)):
            r0, r1, r2 = self.vertices[0][i-1], self.vertices[1][i-1], self.vertices[2][i-1]
            new_r0 = (r0[0] + translation_step, r0[1], r0[2])
            new_r1 = (r1[0] + translation_step, r1[1], r1[2])
            new_r2 = (r2[0] + translation_step, r2[1], r2[2])
            self.vertices[0].append(new_r0)
            self.vertices[1].append(new_r1)
            self.vertices[2].append(new_r2)

    def jacobian(self,
                 q0: float,
                 q1: float) -> np.ndarray:
        J = np.array([[-self.L0 * np.sin(q0) - self.L1 * np.sin(q0 + q1), -self.L1 * np.sin(q0 + q1)],
                      [self.L0 * np.cos(q0) + self.L1 * np.cos(q0 + q1), self.L1 * np.cos(q0 + q1)]])

        return J
        
