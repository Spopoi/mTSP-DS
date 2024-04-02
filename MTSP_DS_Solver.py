import itertools

import gurobipy as gp
import numpy as np
from gurobipy import GRB
from Node import Node
from Customer import Customer
from TourUtils import get_k_value, getVisitedNodesIndex, generate_sub_tours_indexes, tourToTuple
from Truck import Truck
from DroneStation import DroneStation
from Location import Location
from Depot import Depot
import matplotlib.pyplot as plt


class MTSP_DS_Solver:
    maxLocationBound = 150

    def __init__(self, n, m, Dn=2, Kn=1, C=None, alpha=1.2, eps=100):
        self.n = n  # customers
        self.m = m  # drone stations
        self.Dn = Dn  # num of drones per drone station
        self.Kn = Kn  # trucks
        self.C = C if C is not None else m  # max number of actionable drone stations (<= m)
        self.alpha = alpha  # drone velocity factor relative to truck speed (>1 means drone faster than truck)
        self.eps = eps  # eps = 200  # max drone distance

        self.Vn = np.arange(1, self.n + 1)
        self.K = np.arange(1, self.Kn + 1)
        self.Vs = np.arange(self.n + 1, self.n + self.m + 1)
        self.D = np.arange(1, self.Dn + 1)

        self.v = []  # nodes
        # self.v = np.empty(self.n + self.m + 2, dtype=Node)
        self.k = np.empty(self.Kn, dtype=Node)
        # self.k = []  # trucks

        self.initNodes()
        # self.plotNodes()

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
        # starting depot
        self.v.append(Depot(0, Location(0, 0)))
        # customers:
        for i in self.Vn:
            self.v.append(Customer(i, self.rand_location()))
        # Drone stations:
        for j in self.Vs:
            self.v.append(DroneStation(j, self.rand_location(), self.Dn))
        # Ending depot
        self.v.append(Depot(self.n+self.m+1, Location(0, 0)))

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

        self.showOptimisationLog(False)
        self.model.Params.lazyConstraints = 1
        # self.model.setParam('OutputFlag', 0)

    def showOptimisationLog(self, activate):
        if activate:
            self.model.setParam('OutputFlag', 1)
        else:
            self.model.setParam('OutputFlag', 0)

    def getTrucksTour_callback(self):
        return self._getTrucksTour(lambda var: self.model.cbGetSolution(var) == 1)

    def getTrucksTour(self):
        return self._getTrucksTour(lambda var: var.x == 1)

    def _getTrucksTour(self, decision_checker):
        k_var_lists = {}
        truck_k_tour = []
        for var in self.model._vars:
            if "x_k_ij" in var.varName:
                k = get_k_value(var.varName)  # Extract k value from variable name
                if k not in k_var_lists:
                    k_var_lists[k] = []
                if decision_checker(var):
                    k_var_lists[k].append(var)
        for k, var_list in k_var_lists.items():
            truck_k_tour.append(var_list)
        return truck_k_tour

    def subtourelim(self, model, where):
        if where == GRB.Callback.MIPSOL:
            tours = self.getTrucksTour_callback()
            truck_index = 1
            x_k_ij = model._edges
            for truck_tour in tours:
                node_indexes = getVisitedNodesIndex(truck_tour)
                sub_tours_indexes = generate_sub_tours_indexes(node_indexes)
                # Constraint (13)
                for S in sub_tours_indexes:
                    model.cbLazy(
                        gp.quicksum(gp.quicksum(x_k_ij[truck_index, i, j] for j in S if i != j) for i in S)
                        <= len(S) - 1)
                    model.update()
                truck_index += 1

    def getSolution(self):
        return self.model.ObjVal

    def plot_arrows_between_nodes(self, data):
        for edge in data:
            start_node = self.v[edge[0][0]]
            end_node = self.v[edge[0][1]]
            plt.arrow(start_node.location.x, start_node.location.y,
                      end_node.location.x - start_node.location.x,
                      end_node.location.y - start_node.location.y,
                      head_width=0.2, head_length=0.2, fc='blue', ec='blue')
            plt.text(start_node.location.x, start_node.location.y, f'{start_node.index}', horizontalalignment='center')

        plt.scatter([node.location.x for node in self.v],
                    [node.location.y for node in self.v], color='red', zorder=1)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.show()

    def plotTours(self):
        tours = self.getTrucksTour()
        for tour in tours:
            tuples_tour = tourToTuple(tour)
            self.plot_arrows_between_nodes(tuples_tour)

    def getExecTime(self):
        return self.model.Runtime

    def printExecutionLog(self):
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

        # self.plotTours()

    def solve(self):
        self.model.optimize(self.subtourelim)
