class Drone:
    def __init__(self, index, location, speed_factor=1.5, eps=100):
        self.index = index
        self.speed_factor = speed_factor
        self.location = location
        self.eps = eps  # max drone distance

    def __repr__(self):
        return (
            f"Drone {self.index} at {self.location}, "
            f"Speed Factor: {self.speed_factor}, "
            f"Max Drone Distance: {self.eps}"
        )

