import itertools

import gurobipy as gp
import numpy as np
from gurobipy import GRB

from Node import Node
from Customer import Customer
from TourUtils import get_k_value, getVisitedNodesIndex, generate_sub_tours_indexes
from Truck import Truck
from DroneStation import DroneStation
from Location import Location
from Depot import Depot
import matplotlib.pyplot as plt


class MTSPSolver:
    default_Dn_value = 2
    default_Kn_value = 2
    default_alpha_value = 1.2
    default_eps_value = 200
    maxLocationBound = 200

    def __init__(self, n, m, Dn=None, Kn=None, C=None, alpha=None, eps=None):
        self.n = n  # customers
        self.m = m  # drone stations
        self.Dn = Dn if Dn is not None else self.default_Dn_value  # num of drones per drone station
        self.Kn = Kn if Kn is not None else self.default_Kn_value  # trucks
        self.C = C if C is not None else m  # max number of actionable drone stations (<= m)
        self.alpha = alpha if alpha is not None else self.default_alpha_value
        # drone velocity factor relative to truck speed (>1 means drone faster than truck)
        self.eps = eps if eps is not None else self.default_eps_value  # eps = 200  # max drone distance

        self.Vn = np.arange(1, self.n + 1)
        self.K = np.arange(1, self.Kn + 1)
        self.Vs = np.arange(self.n + 1, self.n + self.m + 1)
        self.D = np.arange(1, self.Dn + 1)

        self.v = []  # nodes
        # self.v = np.empty(self.n + self.m + 2, dtype=Node)
        self.k = np.empty(self.Kn, dtype=Node)
        # self.k = []  # trucks

        self.initNodes()
        self.plotNodes()

        self.V = np.arange(len(self.v))
        self.Vl = self.V[:-1]
        self.Vr = self.V[1:]
        self.H = itertools.chain(self.Vn, self.Vs)  # Vn u Vs

        self.t_ij, self.t_ij_drone = self.calculate_distance_matrices()

        self.model = gp.Model("mTSP-DS")

        # DECISION VARIABLES
        # Makespan
        self.tau = self.model.addVar(vtype=GRB.CONTINUOUS, name="tau")

        # truck k traverse edge (i,j)
        self.x_k_ij = self.model.addVars([(k, i, j) for k in self.K for i in self.Vl for j in self.Vr],
                                         vtype=GRB.BINARY, name="x_k_ij")

        # DS activation
        self.z_s = self.model.addVars(self.Vs, vtype=GRB.INTEGER, name="z_s")

        # time truck k arrives at node i
        self.a_ki = self.model.addVars([(i, j) for i in self.K for j in self.V], vtype=GRB.CONTINUOUS, name="a_ki")

        # customer j served by drone d from drone station s
        self.y_d_sj = self.model.addVars([(d, s, j) for d in self.D for s in self.Vs for j in self.Vn],
                                         vtype=GRB.BINARY, name="y_d_sj")
        self.initModel()

    def initNodes(self):

        self.k[:] = [Truck(i, Location(0, 0)) for i in range(self.Kn)]
        # for i in range(self.Kn):
        # self.k.append(Truck(i, Location(0, 0)))
        # starting depot
        self.v.append(Depot(0, Location(0, 0)))
        # customers:
        for i in self.Vn:
            self.v.append(Customer(i, self.rand_location()))
        # Drone stations:
        for j in self.Vs:
            self.v.append(DroneStation(j, self.rand_location(), self.Dn))
        # Ending depot
        self.v.append(Depot(1, Location(0, 0)))

    def rand_location(self):
        rand_x = np.random.randint(1, self.maxLocationBound)
        rand_y = np.random.randint(1, self.maxLocationBound)
        return Location(rand_x, rand_y)

    def calculate_distance_matrices(self):
        num_nodes = len(self.v)
        t_ij = np.zeros((num_nodes, num_nodes))
        t_ij_drone = np.zeros((num_nodes, num_nodes))

        for i in range(num_nodes):
            for j in range(num_nodes):
                t_ij[i][j] = self.v[i].node_distance(self.v[j])
                t_ij_drone[i][j] = self.v[i].node_distance(self.v[j]) / self.alpha
        return t_ij, t_ij_drone

    def plotNodes(self):
        x_values = [node.location.x for node in self.v]
        y_values = [node.location.y for node in self.v]

        colors = ['blue' if isinstance(node, Customer) else 'red' if isinstance(node, DroneStation) else 'black'
                  for node in self.v]

        plt.scatter(x_values, y_values, color=colors, label="Nodes")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Nodes plot")

        for node in self.v[:-1]:
            plt.annotate(str(node.index), (node.location.x, node.location.y), textcoords="offset points",
                         xytext=(0, 10),
                         ha='center')

        blue_patch = plt.Line2D([0], [0], marker='o', color='w', label='Customer', markerfacecolor='blue',
                                markersize=10)
        red_patch = plt.Line2D([0], [0], marker='o', color='w', label='Drone Station', markerfacecolor='red',
                               markersize=10)
        black_patch = plt.Line2D([0], [0], marker='o', color='w', label='Depot', markerfacecolor='black', markersize=10)
        plt.legend(handles=[black_patch, red_patch, blue_patch])
        plt.show()

    def initModel(self):
        self.model.update()
        self.model.setObjective(self.tau, sense=GRB.MINIMIZE)

        # CONSTRAINTS
        # Constraint (2)
        self.model.addConstrs((self.a_ki[k, 1 + self.n + self.m] <= self.tau for k in self.K), name="(2)")

        # Constraint (3)
        self.model.addConstrs((self.a_ki[k, s] + gp.quicksum(
            2 * self.t_ij_drone[s, j] * self.y_d_sj[d, s, j] for j in self.Vn) <= self.tau
                               for k in self.K for s in self.Vs for d in self.D), name="(3)")

        # Constraint (4)
        self.model.addConstrs(
            (gp.quicksum(gp.quicksum(self.x_k_ij[k, i, j] for i in self.Vl if i != j) for k in self.K) +
             gp.quicksum(gp.quicksum(self.y_d_sj[d, s, j] for d in self.D) for s in self.Vs) == 1 for j in self.Vn),
            name="(4)")

        # Constraint (5.1)
        self.model.addConstrs((gp.quicksum(self.x_k_ij[k, 0, j] for j in self.Vn) == 1 for k in self.K), name="(5.1)")

        # Constraint (5.2)
        self.model.addConstrs(
            (gp.quicksum(self.x_k_ij[k, i, 1 + self.n + self.m] for i in self.Vn) == 1 for k in self.K), name="(5.2)")
        # i non dovrebbe essere presa su Vl e non su Vn? Mi va bene che l'ultimo step del truck è una DS
        # se è poi la DS a servire l'ultimo client...

        # Constraint (6)
        self.model.addConstrs(((gp.quicksum(self.x_k_ij[k, i, h] for i in self.Vl if i != h)
                                - gp.quicksum(self.x_k_ij[k, h, j] for j in self.Vr if h != j) == 0) for h in self.H for
                               k in self.K), name="(6)")

        # Constraint (7)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, s] for i in self.Vl if i != s)
                                           for k in self.K) <= 1 for s in self.Vs), name="(7)")

        # Constraint (8)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, s] for i in self.Vl if i != s)
                                           for k in self.K) == self.z_s[s] for s in self.Vs), name="(8)")

        # Constraint (9)
        self.model.addConstr((gp.quicksum(self.z_s[s] for s in self.Vs) <= self.C), name="(9)")

        # Constraint (10)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.y_d_sj[d, s, j] for j in self.Vn)
                                           for d in self.D) <= self.n * self.z_s[s] for s in self.Vs), name="(10)")

        # Constraint (11)
        self.model.addConstrs((2 * self.t_ij[s, j] * self.y_d_sj[d, s, j] <= self.eps
                               for s in self.Vs for d in self.D for j in self.Vn), name="(11)")

        # Constraint (12)
        M = 1000
        self.model.addConstrs((M * (self.x_k_ij[k, i, j] - 1) + self.a_ki[k, i] + self.t_ij[i, j] <= self.a_ki[k, j]
                               for k in self.K for i in self.Vl for j in self.Vr if i != j), name="(12)")

        # Constraint (14)
        self.model.addConstrs((gp.quicksum(
            gp.quicksum(self.x_k_ij[k, i, j] * self.t_ij[i, j] for j in self.Vr if i != j) for i in self.Vl)
                               <= self.a_ki[k, 1 + self.n + self.m] for k in self.K), name="(14)")

        self.model.update()
        self.model.write("modello.lp")
        self.model._edges = self.x_k_ij
        self.model._vars = self.model.getVars()

        self.model.Params.lazyConstraints = 1

    def getTrucksTour(self):
        k_var_lists = {}
        truck_k_tour = []
        for var in self.model._vars:
            if "x_k_ij" in var.varName:
                k = get_k_value(var.varName)  # Extract k value from variable name
                if k not in k_var_lists:
                    k_var_lists[k] = []
                if self.model.cbGetSolution(var) == 1:
                    k_var_lists[k].append(var)
        for k, var_list in k_var_lists.items():
            truck_k_tour.append(var_list)
            # print(f"Variables for k = {k}: {var_list}")
        return truck_k_tour

    def subtourelim(self, model, where):
        if where == GRB.Callback.MIPSOL:
            tours = self.getTrucksTour()
            truck_index = 1
            x_k_ij = model._edges
            for truck_tour in tours:
                node_indexes = getVisitedNodesIndex(truck_tour)
                sub_tours_indexes = generate_sub_tours_indexes(node_indexes)
                # Constraint (13)
                for S in sub_tours_indexes:
                    # print(S)
                    print(gp.quicksum(gp.quicksum(x_k_ij[truck_index, i, j] for j in S if i != j) for i in S) <= len(
                        S) - 1)
                    model.cbLazy(
                        gp.quicksum(gp.quicksum(x_k_ij[truck_index, i, j] for j in S if i != j) for i in S)
                        <= len(S) - 1)
                    model.update()
                truck_index += 1


    def solve(self):
        self.model.optimize(self.subtourelim)

        print("---------model.status--------")
        print(self.model.status)
        # https://www.gurobi.com/documentation/9.5/refman/optimization_status_codes.html

        print("-----------model.Runtime-----------------")
        print(self.model.Runtime)
        # https://www.gurobi.com/documentation/9.5/refman/runtime.html

        print("-------model.ObjVal----------")
        print(self.model.ObjVal)
        # https://www.gurobi.com/documentation/9.5/refman/objval.html#attr:ObjVal

        for var in self.model.getVars():
            if var.x == 1:
                print(f"{var.varName}: {var.x}")

        self.model.write("modello_finale.lp")


if __name__ == "__main__":
    solver = MTSPSolver(6, 3)
    solver.solve()

#
# def plot_nodes(nodes):
#     x_values = [node.location.x for node in nodes]
#     y_values = [node.location.y for node in nodes]
#
#     colors = ['blue' if isinstance(node, Customer) else 'red' if isinstance(node, DroneStation) else 'black' for node in
#               nodes]
#
#     plt.scatter(x_values, y_values, color=colors, label="Nodes")
#     plt.xlabel("X")
#     plt.ylabel("Y")
#     plt.title("Nodes plot")
#
#     for node in nodes[:-1]:
#         plt.annotate(str(node.index), (node.location.x, node.location.y), textcoords="offset points", xytext=(0, 10),
#                      ha='center')
#
#     blue_patch = plt.Line2D([0], [0], marker='o', color='w', label='Customer', markerfacecolor='blue', markersize=10)
#     red_patch = plt.Line2D([0], [0], marker='o', color='w', label='Drone Station', markerfacecolor='red', markersize=10)
#     black_patch = plt.Line2D([0], [0], marker='o', color='w', label='Depot', markerfacecolor='black', markersize=10)
#     plt.legend(handles=[black_patch, red_patch, blue_patch])
#     plt.show()
#
#
# def generate_sub_tours_indexes(v):
#     sub_tours_indexes = []
#     for sub_tour_length in range(2, len(v) - 1):
#         for combo in itertools.combinations(v[1:-1], sub_tour_length):
#             sub_tours_indexes.append(list(combo))
#     return sub_tours_indexes
#
#
# def rand_location(maxLocationBound):
#     rand_x = np.random.randint(1, maxLocationBound)
#     rand_y = np.random.randint(1, maxLocationBound)
#     return Location(rand_x, rand_y)
#
#
# def calculate_distance_matrices(v, alpha):
#     num_nodes = len(v)
#     t_ij = np.zeros((num_nodes, num_nodes))
#     t_ij_drone = np.zeros((num_nodes, num_nodes))
#
#     for i in range(num_nodes):
#         for j in range(num_nodes):
#             t_ij[i][j] = v[i].node_distance(v[j])
#             t_ij_drone[i][j] = v[i].node_distance(v[j]) / alpha
#     return t_ij, t_ij_drone
#
#
# def get_k_value(var_name):
#     parts = var_name.split("[")
#     return int(parts[1][0])
#
#
# def getTrucksTour(model):
#     k_var_lists = {}
#     truck_k_tour = []
#     for var in model._vars:
#         if "x_k_ij" in var.varName:
#             k = get_k_value(var.varName)  # Extract k value from variable name
#             if k not in k_var_lists:
#                 k_var_lists[k] = []
#             if model.cbGetSolution(var) == 1:
#                 k_var_lists[k].append(var)
#     for k, var_list in k_var_lists.items():
#         truck_k_tour.append(var_list)
#         # print(f"Variables for k = {k}: {var_list}")
#     return truck_k_tour
#
#
# def getVisitedNodesIndex(tour):
#     visited_nodes = []
#     for var in tour:
#         parts = var.varName.split(",")
#         starting_node = int(parts[1])
#         ending_node = int(parts[2][:-1])
#         if starting_node not in visited_nodes:
#             visited_nodes.append(starting_node)
#         if ending_node not in visited_nodes:
#             visited_nodes.append(ending_node)
#     return sorted(visited_nodes)
#
#
# def subtourelim(model, where):
#     if where == GRB.Callback.MIPSOL:
#         tours = getTrucksTour(model)
#         truck_index = 1
#         x_k_ij = model._edges
#         for truck_tour in tours:
#             node_indexes = getVisitedNodesIndex(truck_tour)
#             sub_tours_indexes = generate_sub_tours_indexes(node_indexes)
#             # Constraint (13)
#             for S in sub_tours_indexes:
#                 # print(S)
#                 # print(gp.quicksum(gp.quicksum(x_k_ij[truck_index, i, j] for j in S if i != j) for i in S) <= len(S) - 1)
#                 model.cbLazy(gp.quicksum(gp.quicksum(x_k_ij[truck_index, i, j] for j in S if i != j) for i in S)
#                              <= len(S) - 1)
#             truck_index += 1
#
#
# def solve():
#     # Parameters
#     n = 6  # customers
#     m = 2  # drone stations
#     Dn = 1  # num of drones per drone station
#     Kn = 2  # trucks
#     C = 1  # max number of actionable drone stations (<= m)
#     alpha = 1.2  # drone velocity factor relative to truck speed (>1 means drone faster than truck)
#     eps = 200  # max drone distance
#
#     custom_setup = False
#     maxLocationBound = 200
#
#     # indexes
#     Vn = range(1, n + 1)
#     K = range(1, Kn + 1)
#     Vs = range(n + 1, n + m + 1)
#     D = range(1, Dn + 1)
#
#     v = []  # nodes
#     k = []  # trucks
#
#     for i in range(Kn):
#         k.append(Truck(i, Location(0, 0)))
#
#     if custom_setup:
#         # starting depot
#         v.append(Depot(0, Location(0, 0)))
#         # customers:
#         v.append(Customer(1, Location(10, 10)))
#         v.append(Customer(2, Location(20, 10)))
#         v.append(Customer(3, Location(40, 80)))
#         v.append(Customer(4, Location(100, 60)))
#         # Drone stations:
#         v.append(DroneStation(1, Location(10, 30), Dn))
#         v.append(DroneStation(2, Location(60, 40), Dn))
#         # Ending depot
#         v.append(Depot(1, Location(0, 0)))
#
#     else:
#         # starting depot
#         v.append(Depot(0, Location(0, 0)))
#         # customers:
#         for i in Vn:
#             v.append(Customer(i, rand_location(maxLocationBound)))
#         # Drone stations:
#         for j in Vs:
#             v.append(DroneStation(j, rand_location(maxLocationBound), Dn))
#         # Ending depot
#         v.append(Depot(1, Location(0, 0)))
#
#     plot_nodes(v)
#
#     V = range(len(v))
#     Vl = V[:-1]
#     Vr = V[1:]
#     H = itertools.chain(Vn, Vs)  # Vn u Vs
#
#     t_ij, t_ij_drone = calculate_distance_matrices(v, alpha)
#
#     model = gp.Model("mTSP-DS")
#
#     # DECISION VARIABLES
#     # Makespan
#     tau = model.addVar(vtype=GRB.CONTINUOUS, name="tau")
#
#     # truck k traverse edge (i,j)
#     x_k_ij = model.addVars([(k, i, j) for k in K for i in Vl for j in Vr],
#                            vtype=GRB.BINARY, name="x_k_ij")
#
#     # DS activation
#     z_s = model.addVars(Vs, vtype=GRB.INTEGER, name="z_s")
#
#     # time truck k arrives at node i
#     a_ki = model.addVars([(i, j) for i in K for j in V], vtype=GRB.CONTINUOUS, name="a_ki")
#
#     # customer j served by drone d from drone station s
#     y_d_sj = model.addVars([(d, s, j) for d in D for s in Vs for j in Vn],
#                            vtype=GRB.BINARY, name="y_d_sj")
#
#     model.update()
#     model.setObjective(tau, sense=GRB.MINIMIZE)
#
#     # CONSTRAINTS
#     # Constraint (2)
#     model.addConstrs((a_ki[k, 1 + n + m] <= tau for k in K), name="(2)")
#
#     # Constraint (3)
#     model.addConstrs((a_ki[k, s] + gp.quicksum(2 * t_ij_drone[s, j] * y_d_sj[d, s, j] for j in Vn) <= tau
#                       for k in K for s in Vs for d in D), name="(3)")
#
#     # Constraint (4)
#     model.addConstrs((gp.quicksum(gp.quicksum(x_k_ij[k, i, j] for i in Vl if i != j) for k in K) +
#                       gp.quicksum(gp.quicksum(y_d_sj[d, s, j] for d in D) for s in Vs) == 1 for j in Vn), name="(4)")
#
#     # Constraint (5.1)
#     model.addConstrs((gp.quicksum(x_k_ij[k, 0, j] for j in Vn) == 1 for k in K), name="(5.1)")
#
#     # Constraint (5.2)
#     model.addConstrs((gp.quicksum(x_k_ij[k, i, 1 + n + m] for i in Vn) == 1 for k in K), name="(5.2)")
#     # i non dovrebbe essere presa su Vl e non su Vn? Mi va bene che l'ultimo step del truck è una DS
#     # se è poi la DS a servire l'ultimo client...
#
#     # Constraint (6)
#     model.addConstrs(((gp.quicksum(x_k_ij[k, i, h] for i in Vl if i != h)
#                        - gp.quicksum(x_k_ij[k, h, j] for j in Vr if h != j) == 0) for h in H for k in K), name="(6)")
#
#     # Constraint (7)
#     model.addConstrs((gp.quicksum(gp.quicksum(x_k_ij[k, i, s] for i in Vl if i != s)
#                                   for k in K) <= 1 for s in Vs), name="(7)")
#
#     # Constraint (8)
#     model.addConstrs((gp.quicksum(gp.quicksum(x_k_ij[k, i, s] for i in Vl if i != s)
#                                   for k in K) == z_s[s] for s in Vs), name="(8)")
#
#     # Constraint (9)
#     model.addConstr((gp.quicksum(z_s[s] for s in Vs) <= C), name="(9)")
#
#     # Constraint (10)
#     model.addConstrs((gp.quicksum(gp.quicksum(y_d_sj[d, s, j] for j in Vn)
#                                   for d in D) <= n * z_s[s] for s in Vs), name="(10)")
#
#     # Constraint (11)
#     model.addConstrs((2 * t_ij[s, j] * y_d_sj[d, s, j] <= eps
#                       for s in Vs for d in D for j in Vn), name="(11)")
#
#     # Constraint (12)
#     M = 1000
#     model.addConstrs((M * (x_k_ij[k, i, j] - 1) + a_ki[k, i] + t_ij[i, j] <= a_ki[k, j]
#                       for k in K for i in Vl for j in Vr if i != j), name="(12)")
#
#     # Constraint (14)
#     model.addConstrs((gp.quicksum(gp.quicksum(x_k_ij[k, i, j] * t_ij[i, j] for j in Vr if i != j) for i in Vl)
#                       <= a_ki[k, 1 + n + m] for k in K), name="(14)")
#
#     model.update()
#     model.write("modello.lp")
#     model._edges = x_k_ij
#     model._vars = model.getVars()
#
#     model.Params.lazyConstraints = 1
#     model.optimize(subtourelim)
#
#     print("---------model.status--------")
#     print(model.status)
#     # https://www.gurobi.com/documentation/9.5/refman/optimization_status_codes.html
#
#     print("-----------model.Runtime-----------------")
#     print(model.Runtime)
#     # https://www.gurobi.com/documentation/9.5/refman/runtime.html
#
#     print("-------model.ObjVal----------")
#     print(model.ObjVal)
#     # https://www.gurobi.com/documentation/9.5/refman/objval.html#attr:ObjVal
#
#     for var in model.getVars():
#         print(f"{var.varName}: {var.x}")
#
#
# if __name__ == "__main__":
#     solve()