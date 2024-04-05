from Drone import Drone
from Node import Node, NodeType


class DroneStation(Node):
    def __init__(self, index, location, drone_number):
        super().__init__(location, NodeType.DRONE_STATION, index)
        self.drone_number = drone_number
        self.drones = [Drone(i, location) for i in range(drone_number)]

    def __repr__(self):
        drones_info = "\n".join([repr(drone) for drone in self.drones])
        return (
            f"Station Index: {self.index}\n"
            f"{super().__repr__()}\n"
            f"Drones:\n{drones_info}"
        )

    def get_num_drones(self):
        return len(self.drones)
