import gurobipy as gp
from gurobipy import GRB
from Customer import Customer
from Truck import Truck
from Drone import Drone
from DroneStation import DroneStation
from Location import Location
from Depot import Depot
import matplotlib.pyplot as plt


def plot_nodes(nodes):
    # Todo: Spezzare i nodi e plottarli separatamente
    x_values = [node.location.x for node in nodes]
    y_values = [node.location.y for node in nodes]

    colors = ['blue' if isinstance(node, Customer) else 'red' for node in nodes]

    plt.scatter(x_values, y_values, color=colors, label="Nodes")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Nodes plot")

    for node in nodes:
        plt.annotate(str(node.index), (node.location.x, node.location.y), textcoords="offset points", xytext=(0, 10),
                     ha='center')

    blue_patch = plt.Line2D([0], [0], marker='o', color='w', label='Customer', markerfacecolor='blue', markersize=10)
    red_patch = plt.Line2D([0], [0], marker='o', color='w', label='Drone Station', markerfacecolor='red', markersize=10)
    plt.legend(handles=[blue_patch, red_patch])
    plt.show()


def solve():
    # Parameters
    n = 4  # customers
    m = 2  # drone stations
    Dn = 1  # num of drones per drone station
    Kn = 2  # trucks
    C = 1  # max number of actionable drone stations (<= m)
    alpha = 0.5  # drone velocity factor relative to truck speed
    eps = 100  # max drone distance
    custom_setup = True

    v = []  # nodes
    k = []  # trucks

    if custom_setup:
        # starting depot
        v.append(Depot(0, Location(0, 0)))
        # customers:
        v.append(Customer(1, Location(10, 10)))
        v.append(Customer(2, Location(20, 10)))
        v.append(Customer(3, Location(40, 80)))
        v.append(Customer(4, Location(100, 60)))
        # Drone stations:
        v.append(DroneStation(1, Location(10, 30), Dn))
        v.append(DroneStation(2, Location(60, 40), Dn))
        # Ending depot
        v.append(Depot(1, Location(0, 0)))
        plot_nodes(v)
        for i in range(Kn):
            k.append(Truck(i, Location(0, 0)))
    vl = v[1:]  # nodes \ initialDepot
    vr = v[:-1]  # node \ endingDepot

    model = gp.Model("mTSP-DS")

    # Decision variables
    # Makespan
    tau = model.addVar(vtype=GRB.CONTINUOUS, name="tau")

    # DS activation
    z = model.addVars(m, vtype=GRB.INTEGER, name="z")

    # truck k traverse edge (i,j)
    x_k_ij = model.addVars([(k, i, j) for k in range(Kn) for i in range(n-1) for j in range(n-1)],
                           vtype=GRB.BINARY, name="x_k_ij")

    # time truck k arrives at node i
    a_ki = model.addVars([(i, j) for i in range(Kn) for j in range(len(v))], vtype=GRB.CONTINUOUS, name="a_ki")

    # customer j served by drone d from drone station s
    y_d_sj = model.addVars([(d, s, j) for d in range(Dn) for s in range(m) for j in range(n)],
                           vtype=GRB.BINARY, name="y_d_sj")

    # model.update()
    model.setObjective(tau, sense=GRB.MINIMIZE)

    model.update()
    model.write("modello.lp")


if __name__ == "__main__":
    solve()