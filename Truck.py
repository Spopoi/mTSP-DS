class Truck:

    def __init__(self, index, location):
        self.index = index
        self.location = location

    def __repr__(self):
        return (
            f"Truck index: {self.index}\n"
            f"Truck location: {self.location}\n"
        )
