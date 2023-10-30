import math


class Location:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"({self.x},{self.y})"

    def euclidean_distance(self, other_location):
        delta_x = self.x - other_location.x
        delt_y = self.y - other_location.y
        return math.sqrt(delta_x * delta_x + delt_y * delt_y)
