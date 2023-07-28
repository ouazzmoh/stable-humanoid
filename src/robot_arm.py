from typing import List, Tuple


class RobotArm:
    def __init__(self,
                 vertices: List[Tuple[float, float, float]]):
        self.vertices = vertices
