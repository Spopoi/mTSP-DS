from Drone import Drone


class DroneStation:
    def __init__(self, index, location, drone_number):
        self.index = index
        self.location = location
        self.drone_number = drone_number
        self.drones = []
        for i in range(drone_number):
            self.drones.append(Drone(i, location))

    def __repr__(self):
        drones_info = "\n".join([repr(drone) for drone in self.drones])
        return f"Station Index: {self.index}\nStation Location: {self.location}\nDrones:\n{drones_info}"

    def get_num_drones(self):
        return len(self.drones)
