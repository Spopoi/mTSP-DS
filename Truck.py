class Truck:
    def __init__(self, index, location):
        self.index = index
        self.x = location.x
        self.y = location.y

    def __repr__(self):
        return (
            f"Truck index: {self.index}, "
            f"Truck location: ({self.x}, {self.y})"
        )
