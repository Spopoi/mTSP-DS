import gurobipy as gp
import numpy as np
from Node import Node
from Customer import Customer
from TourUtils import get_k_value, tourToTuple, _getTrucksTour, plotNodes, getTrucksTour
from Truck import Truck
from DroneStation import DroneStation
from Location import Location, rand_location
from Depot import Depot
import matplotlib.pyplot as plt


class MTSP_DS_Solver:

    def __init__(self, n, m, Dn=2, Kn=1, C=None, alpha=1.2, eps=100, nodes=None, custom_locations=None):
        self.n = n  # customers
        self.m = m  # drone stations
        self.Dn = Dn  # num of drones per drone station
        self.Kn = Kn  # trucks
        self.C = C if C is not None else m  # max number of actionable drone stations (<= m)
        self.alpha = alpha  # drone velocity factor relative to truck speed (>1 means drone faster than truck)
        self.eps = eps  # eps = 200  # max drone distance

        self.K = np.arange(1, self.Kn + 1)
        self.Vn = np.arange(1, self.n + 1)
        self.Vs = np.arange(self.n + 1, self.n + self.m + 1)
        self.D = np.arange(1, self.Dn + 1)

        self.v = []  # nodes
        self.k = np.empty(self.Kn, dtype=Node)

        self.model = gp.Model("mTSP-DS")

        self.k[:] = [Truck(i + 1, Location(0, 0)) for i in range(self.Kn)]
        # starting depot
        self.v.append(Depot(0, Location(0, 0)))

        if nodes is not None:
            self.v.extend([node for node in nodes])
        elif custom_locations is not None:
            self.nodes_init(custom_locations)
        else:
            self.random_init()
        # Ending depot
        self.v.append(Depot(self.n + self.m + 1, Location(0, 0)))

        # print("v= ", self.v)
        self.t_ij, self.t_ij_drone = self.calculate_distance_matrices()
        # self.plotNodes()

    def nodes_init(self, locations):
        # customers:
        for i in self.Vn:
            self.v.append(Customer(i, locations[i - 1]))
        # Drone stations:
        for j in self.Vs:
            self.v.append(DroneStation(j, locations[j - 1], self.Dn))

    def random_init(self):
        # customers:
        for i in self.Vn:
            self.v.append(Customer(i, rand_location()))
        # Drone stations:
        for j in self.Vs:
            self.v.append(DroneStation(j, rand_location(), self.Dn))
        # self.plotNodes()

    def save_nodes_location_to_file(self, filepath: str):
        with open(filepath, 'a') as file:
            for node in self.v[1:]:
                location = node.location
                file.write(f"{location} ")
            file.write("\n")

    def calculate_distance_matrices(self):
        num_nodes = len(self.v)
        t_ij = np.zeros((num_nodes, num_nodes))
        t_ij_drone = np.zeros((num_nodes, num_nodes))

        for i in range(num_nodes):
            for j in range(num_nodes):
                t_ij[i][j] = self.v[i].node_distance(self.v[j])
                t_ij_drone[i][j] = self.v[i].node_distance(self.v[j]) / self.alpha
        return t_ij, t_ij_drone

    def showOptimisationLog(self, activate):
        if activate:
            self.model.setParam('OutputFlag', 1)
        else:
            self.model.setParam('OutputFlag', 0)

    def NodesTour(self):
        nodes_tours = []
        tours = getTrucksTour(self.model)
        for tour in tours:
            tour_tuples = tourToTuple(tour)
            tour_node = []
            for tour_tuple in tour_tuples:
                tour_node.append(self.v[tour_tuple[0]])
            nodes_tours.append(tour_node)
        return nodes_tours

    def getSolution(self):
        return self.model.ObjVal

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
