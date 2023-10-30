import gurobipy as gp
from gurobipy import GRB
from Customer import Customer
from Truck import Truck
from Drone import Drone
from DroneStation import DroneStation
from Location import Location

# Parameters
n = 10  # customers
m = 1  # drone stations
Kn = 1  # trucks
C = 1
alpha = 0.5
eps = 100  # metri
custom_setup = True

v = []  # nodes


def init():
    if custom_setup:
        # starting depot
        v.append(0)
        # customers:
        v.append(Customer(1, Location(10, 10)))
        v.append(Customer(2, Location(20, 10)))
        v.append(Customer(3, Location(40, -10)))
        v.append(Customer(4, Location(100, 60)))
        # Drone stations:
        v.append(DroneStation(1, Location(30, 30), 2))
        v.append(DroneStation(2, Location(-10, 40), 1))
        # Ending depot
        v.append(1)


def solve():
    model = gp.Model("mTSP-DS")

    # Variabili di decisione
    tau = model.addVar(vtype=GRB.CONTINUOUS, name="tau")

    for k in range(Kn):
        x2 = model.addVar(vtype=GRB.INTEGER, name="x2")


def test():
    loc1 = Location(4, 2)
    loc2 = Location(9, 1)
    drone1 = Drone(1, loc2)
    drone2 = Drone(2, loc1)
    print(drone2)
    array_droni = [drone1, drone2]

    station = DroneStation(index=1, location="Station A", drones=array_droni)
    print(station)


if __name__ == "__main__":
    solve()
    # test()
    init()
    for i in range(len(v)):
        print(str(v[i]) + " ")

