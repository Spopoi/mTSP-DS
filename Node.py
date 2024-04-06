from enum import Enum


class NodeType(Enum):
    CUSTOMER = "Customer"
    DRONE_STATION = "DroneStation"
    DEPOT = "Depot"


class Node:

    def __init__(self, location, node_type, index):
        self.location = location
        self.node_type = node_type
        self.index = index

    def __repr__(self):
        return (
            f"Node index: {self.index}, "
            f"Node Location: {self.location}, "
            f"Node Type: {self.node_type.value}"
        )

    def node_distance(self, other_node):
        return self.location.euclidean_distance(other_node.location)
