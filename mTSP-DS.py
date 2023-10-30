import gurobipy as gp
from gurobipy import GRB
from Customer import Customer
from Truck import Truck
from Drone import Drone
from DroneStation import DroneStation
from Location import Location
import matplotlib.pyplot as plt

# Parameters
n = 4  # customers
m = 2  # drone stations
Kn = 1  # trucks
C = 1
alpha = 0.5
eps = 100  # metri
custom_setup = True

v = []  # nodes


def init():
    if custom_setup:
        # starting depot
        # v.append(Location(0,0))
        # customers:
        v.append(Customer(1, Location(10, 10)))
        v.append(Customer(2, Location(20, 10)))
        v.append(Customer(3, Location(40, -10)))
        v.append(Customer(4, Location(100, 60)))
        # Drone stations:
        v.append(DroneStation(1, Location(30, 30), 2))
        v.append(DroneStation(2, Location(-10, 40), 1))
        # Ending depot
        # v.append(Location(0, 0))
        plot_nodes(v)


def plot_nodes(nodes):
    # Todo: Spezzare i nodi e plottarli separatamente
    x_values = [node.location.x for node in nodes]
    y_values = [node.location.y for node in nodes]

    colors = ['blue' if isinstance(node, Customer) else 'red' for node in nodes]

    plt.scatter(x_values, y_values, color=colors, label="Nodes")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Plot dei Nodi")

    for node in nodes:
        plt.annotate(str(node.index), (node.location.x, node.location.y), textcoords="offset points", xytext=(0, 10),
                     ha='center')

    blue_patch = plt.Line2D([0], [0], marker='o', color='w', label='Customer', markerfacecolor='blue', markersize=10)
    red_patch = plt.Line2D([0], [0], marker='o', color='w', label='Drone Station', markerfacecolor='red', markersize=10)
    plt.legend(handles=[blue_patch, red_patch])
    plt.show()


def solve():
    model = gp.Model("mTSP-DS")

    # Decision variables
    # Makespan
    tau = model.addVar(vtype=GRB.CONTINUOUS, name="tau")
    # DS activation
    z = model.addVars(m, vtype=GRB.INTEGER, name="z")
    a_ik = model.addVars(Kn, vtype=GRB.CONTINUOUS, name="a_ik")

    for k in range(Kn):
        x2 = model.addVar(vtype=GRB.INTEGER, name="x2")


def test():
    loc1 = Location(4, 2)
    loc2 = Location(9, 1)
    drone1 = Drone(1, loc2)
    drone2 = Drone(2, loc1)
    print(drone2)
    array_droni = [drone1, drone2]

    # station = DroneStation(index=1, location="Station A", drones=array_droni)
    # print(station)


if __name__ == "__main__":
    solve()
    # test()
    init()
    # for i in range(len(v)):
    #     print(str(v[i]) + " ")

