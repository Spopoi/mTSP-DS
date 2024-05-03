from core.Node import NodeType, Node


class Customer(Node):
    def __init__(self, index, location, package_demand=1):
        super().__init__(location, NodeType.CUSTOMER, index)
        self.package_demand = package_demand

    def __repr__(self):
        return super().__repr__()
    # def __repr__(self):
    #     return (
    #         f"{super().__repr__()}, "
    #         f"Customer Package Demand: {self.package_demand}"
    #     )
