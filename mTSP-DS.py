import gurobipy as gp
import numpy as np
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

    colors = ['blue' if isinstance(node, Customer) else 'red' if isinstance(node, DroneStation) else 'black' for node in nodes]

    plt.scatter(x_values, y_values, color=colors, label="Nodes")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Nodes plot")

    for node in nodes[:-1]:
        plt.annotate(str(node.index), (node.location.x, node.location.y), textcoords="offset points", xytext=(0, 10),
                     ha='center')

    blue_patch = plt.Line2D([0], [0], marker='o', color='w', label='Customer', markerfacecolor='blue', markersize=10)
    red_patch = plt.Line2D([0], [0], marker='o', color='w', label='Drone Station', markerfacecolor='red', markersize=10)
    black_patch = plt.Line2D([0], [0], marker='o', color='w', label='Depot', markerfacecolor='black', markersize=10)
    plt.legend(handles=[black_patch, red_patch, blue_patch])
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
        # TODO:  remove comment on the plot function calling
        plot_nodes(v)
        for i in range(Kn):
            k.append(Truck(i, Location(0, 0)))
    vl = v[1:]  # nodes \ initialDepot
    vr = v[:-1]  # node \ endingDepot

    # Define distance matrix (cost) between nodes for both trucks and drones
    t_ij = np.zeros((len(v), len(v)))  # = d_ij = d_ij_drones
    t_ij_drone = np.zeros((len(v), len(v)))
    for i in range(len(v)):
        for j in range(len(v)):
            t_ij[i][j] = v[i].node_distance(v[j])
            t_ij_drone[i][j] = v[i].node_distance(v[j])/alpha

    model = gp.Model("mTSP-DS")

    # DECISION VARIABLES
    # Makespan
    tau = model.addVar(vtype=GRB.CONTINUOUS, name="tau")

    # DS activation
    z_s = model.addVars(m, vtype=GRB.INTEGER, name="z_s")

    # truck k traverse edge (i,j)
    x_k_ij = model.addVars([(k, i, j) for k in range(Kn) for i in range(len(vl)) for j in range(len(vr))],
                           vtype=GRB.BINARY, name="x_k_ij")

    # time truck k arrives at node i
    a_ki = model.addVars([(i, j) for i in range(Kn) for j in range(len(v))], vtype=GRB.CONTINUOUS, name="a_ki")

    # customer j served by drone d from drone station s
    y_d_sj = model.addVars([(d, s, j) for d in range(Dn) for s in range(m) for j in range(n)],
                           vtype=GRB.BINARY, name="y_d_sj")

    model.update()
    model.setObjective(tau, sense=GRB.MINIMIZE)

    # CONSTRAINTS
    # Constraint (2)
    model.addConstrs((a_ki[k, 1 + n + m] <= tau for k in range(Kn)), name="(2)")

    # Constraint (3)
    model.addConstrs((a_ki[k, s] + gp.quicksum(2 * t_ij_drone[s, j] * y_d_sj[d, s, j] for j in range(n)) <= tau
                      for k in range(Kn) for s in range(m) for d in range(Dn)), name="(3)")

    # Constraint (4)
    model.addConstrs((gp.quicksum(gp.quicksum(x_k_ij[k, i, j] for i in range(len(vl)) if i != j-1) for k in range(Kn))
                      + gp.quicksum(gp.quicksum(y_d_sj[d, s, j] for d in range(Dn)) for s in range(m)) == 1
                      for j in range(n)), name="(4)")
    # NB: j-1 visto che le i=0 è il depot mentre j=0 è il primo customer
    # TODO: check i != j-1 non funzia

    # Constraint (5.1)
    model.addConstrs((gp.quicksum(x_k_ij[k, 0, j] for j in range(n)) == 1 for k in range(Kn)), name="(5.1)")

    # Constraint (5.2)
    model.addConstrs((gp.quicksum(x_k_ij[k, i+1, m+n] for i in range(n)) == 1 for k in range(Kn)), name="(5.2)")
    # i non dovrebbe essere presa su Vl e non su Vn? Mi va bene che l'ultimo step del truck è una DS
    # se è poi la DS a servire l'ultimo client...

    # Constraint (6)
    model.addConstrs((gp.quicksum(x_k_ij[k, i, h] for i in range(len(vl)) if h != i+1)
                      - gp.quicksum(x_k_ij[k, 1+h, j] for j in range(len(vr)) if j != h) == 0
                      for k in range(Kn) for h in range(m+n)), name="(6)")
    # il primo constraint non funziona, viene preso x_0_a+1_a che equivale a partenza ed arrivo allo stesso nodo

    # Constraint (7)
    model.addConstrs((gp.quicksum(gp.quicksum(x_k_ij[k, i, s] for i in range(len(vl)) if i != s+1)
                                  for k in range(Kn)) <= 1 for s in range(n, n+m)), name="(7)")

    # Constraint (8)
    model.addConstrs((gp.quicksum(gp.quicksum(x_k_ij[k, i, s] for i in range(len(vl)) if i != s + 1)
                                  for k in range(Kn)) == z_s[s - n] for s in range(n, n + m)), name="(8)")

    # Constraint (9)
    model.addConstr((gp.quicksum(z_s[s] for s in range(m)) <= C), name="(9)")

    # Constraint (10)
    model.addConstrs((gp.quicksum(gp.quicksum(y_d_sj[d, s, j] for j in range(n))
                                  for d in range(Dn)) <= n*z_s[s] for s in range(m)), name="(10)")

    # Constraint (11)
    model.addConstrs((2 * t_ij[1+n+s, 1+j] * y_d_sj[d, s, j] <= eps
                      for s in range(m) for d in range(Dn) for j in range(n)), name="(11)")

    # Constraint (12)
    M = 100
    model.addConstrs((M*(x_k_ij[k, i, j] - 1) + a_ki[k, i] + t_ij[i, j] <= a_ki[k, j]
                      for k in range(Kn) for i in range(len(vl)) for j in range(len(vr)) if i+1 != j), name="(12)")

    # Constraint (13)
    # model.addConstrs((<= ), name="(13)")

    model.update()
    model.write("modello.lp")


if __name__ == "__main__":
    solve()
