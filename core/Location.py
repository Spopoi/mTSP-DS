import math

import numpy as np


def create_custom_location_list(filepath: str):
    custom_location_list = []
    with open(filepath, 'r') as file:
        for line in file:
            line = line.replace("(", "").replace(")", "").strip()
            coordinates = line.split(" ")
            for coordinate in coordinates:
                latitude, longitude = map(float, coordinate.split(","))
                location = Location(latitude, longitude)
                custom_location_list.append(location)
    return custom_location_list


def rand_location(maxLocationBound=150):
    rand_x = np.random.randint(1, maxLocationBound)
    rand_y = np.random.randint(1, maxLocationBound)
    return Location(rand_x, rand_y)


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
