
class Step:
    def __init__(self,
                 start_t : float,
                 end_t : float,
                 z_min : float,
                 z_max : float,
                 shift : float) -> None:
        self.start_t = start_t
        self.end_t = end_t
        self.z_min = z_min
        self.z_max = z_max
        self.shift = shift

