import gurobipy as gp
from gurobipy import GRB
from Customer import Customer
from Truck import Truck
from Drone import Drone
from DroneStation import DroneStation

# Parameters
n = 10  # customers
m = 1  # drone stations
Kn = 1  # trucks
C = 1
alpha = 0.5
eps = 100  # metri


def solve():
    model = gp.Model("mTSP-DS")

    # Variabili di decisione
    tau = model.addVar(vtype=GRB.CONTINUOUS, name="tau")

    for k in range(Kn):
        x2 = model.addVar(vtype=GRB.INTEGER, name="x2")


def test():
    truck = Customer(1, 2)
    truck.print_customer_info()
    drone1 = Drone(1, "A")
    drone2 = Drone(2, "B")
    drone2.print_drone_info()
    array_droni = [drone1, drone2]

    station = DroneStation(index=1, location="Station A", drones=array_droni)
    station.print_station_info()


if __name__ == "__main__":
    solve()
    test()
