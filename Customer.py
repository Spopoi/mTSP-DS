class Customer:

    def __init__(self, index, location, package_demand=1):
        self.index = index
        self.location = location
        self.package_demand = package_demand

    def __repr__(self):
        return (
            f"Customer Index: {self.index}\n"
            f"Customer Location: {self.location}\n"
            f"Customer Package Demand: {self.package_demand}"
        )