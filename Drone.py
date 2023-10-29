class Drone:
      def __init__(self, index, drone_station, speed_factor=1.5, eps=100):
        self.index = index
        self.drone_station = drone_station
        self.speed_factor = speed_factor
        self.eps = eps #max drone distance
