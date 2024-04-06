from Node import Node, NodeType


class Depot(Node):
    def __init__(self, index, location):
        super().__init__(location, NodeType.DEPOT, index)

    def __repr__(self):
        return super().__repr__()
