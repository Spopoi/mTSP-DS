class Drone:
    def __init__(self, index, location, speed_factor=1.5, eps=100):
        self.index = index
        self.speed_factor = speed_factor
        self.location = location
        self.eps = eps  # max drone distance

    def print_drone_info(self):
        print(f"Drone index: {self.index}")
        print(f"Drone location: {self.location}")
        print(f"Drone speed factor: {self.speed_factor}")
        print(f"Max drone distance: {self.eps}")

