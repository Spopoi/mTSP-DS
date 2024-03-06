from Node import Node, NodeType


class Depot(Node):
    def __init__(self, index, location):
        super().__init__(location, node_type=NodeType.DEPOT)
        self.index = index

    def __repr__(self):
        return (
            f"Depot {self.index}:\n"
            f"{super().__repr__()}\n"
        )
