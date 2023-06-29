
class Robot:
    def __init__(self,
                 h: float,
                 foot_dimensions: (float, float),
                 spacing_x: float,
                 spacing_y: float) -> None:
        self.h = h
        self.foot_dimensions = foot_dimensions
        self.spacing_x = spacing_x
        self.spacing_y = spacing_y


