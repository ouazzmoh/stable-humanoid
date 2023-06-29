
class Step:
    def __init__(self,
                 start_time: float,
                 end_time: float,
                 z_min: float,
                 z_max: float,
                 shift: float,
                 orientation: float = 0) -> None:
        self.start_time = start_time
        self.end_time = end_time
        self.z_min = z_min
        self.z_max = z_max
        self.shift = shift
        self.orientation = orientation

