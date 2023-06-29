from step import Step
import numpy as np
from typing import List




class FootstepPlanner:
    def __init__(self,
                 footsteps_x: List[Step],
                 footsteps_y: List[Step]
                 ) -> None:
        self.footsteps_x = footsteps_y
        self.footsteps_y = footsteps_y

    def get_traj(self,
                 time_duration: float,
                 T: float) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray):
        assert(len(self.footsteps_x) == len(self.footsteps_y))
        pass
