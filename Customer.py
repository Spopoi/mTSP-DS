from Node import NodeType, Node


class Customer(Node):
    def __init__(self, index, location, package_demand=1):
        super().__init__(location, node_type=NodeType.CUSTOMER)
        self.index = index
        self.package_demand = package_demand

    def __repr__(self):
        return (
            f"Customer Index: {self.index}\n"
            f"{super().__repr__()}\n"
            f"Customer Package Demand: {self.package_demand}"
        )
