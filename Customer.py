class Customer:

    def __init__(self, index, location, package_demand=1):
        self.index = index
        self.location = location
        self.package_demand = package_demand

    def print_customer_info(self):
        print(f"Customer index: {self.index}")
        print(f"Customer location: {self.location}")
        print(f"Customer package demand: {self.package_demand}")