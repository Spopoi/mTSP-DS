class DroneStation:
    def __init__(self, index, location, drones):
        self.index = index
        self.location = location
        self.drones = drones  # un array di oggetti Drone

    def print_station_info(self):
        print(f"Station Index: {self.index}")
        print(f"Station Location: {self.location}")
        print("Drones:")
        for drone in self.drones:
            print(f"  Drone Index: {drone.index}")
            print(f"  Drone Location: {drone.location}")

    def get_num_drones(self):
        return len(self.drones)
