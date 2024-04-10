import gurobipy as gp
import numpy as np
from Node import Node
from Customer import Customer
from TourUtils import get_k_value, tourToTuple
from Truck import Truck
from DroneStation import DroneStation
from Location import Location
from Depot import Depot
import matplotlib.pyplot as plt


class MTSP_DS_Solver:
    maxLocationBound = 150

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
            # TODO: maybe remove custom_locations
        elif custom_locations is not None:
            self.nodes_init(custom_locations)
        else:
            self.random_init()
        # Ending depot
        self.v.append(Depot(self.n + self.m + 1, Location(0, 0)))
        self.plotNodes()

    def nodes_init(self, locations):
        # customers:
        for i in self.Vn:
            self.v.append(Customer(i, locations[i-1]))
        # Drone stations:
        for j in self.Vs:
            self.v.append(DroneStation(j, locations[j-1], self.Dn))

    def random_init(self):
        # customers:
        for i in self.Vn:
            self.v.append(Customer(i, self.rand_location()))
        # Drone stations:
        for j in self.Vs:
            self.v.append(DroneStation(j, self.rand_location(), self.Dn))
        # self.plotNodes()

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

    def showOptimisationLog(self, activate):
        if activate:
            self.model.setParam('OutputFlag', 1)
        else:
            self.model.setParam('OutputFlag', 0)

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

    def NodesTour(self):
        nodes_tours = []
        tours = self.getTrucksTour()
        for tour in tours:
            tour_tuples = tourToTuple(tour)
            tour_node = []
            for tour_tuple in tour_tuples:
                # node_i = Node.node_index_map[tour_tuple[0]]
                # tour_node.append(node_i)
                tour_node.append(self.v[tour_tuple[0]])
            nodes_tours.append(tour_node)
        # print(nodes_tours)
        return nodes_tours

    def getSolution(self):
        return self.model.ObjVal

    def plotNodes(self, data=None):
        x_values = [node.location.x for node in self.v]
        y_values = [node.location.y for node in self.v]

        node_types = ['customer' if isinstance(node, Customer) else 'drone_station' if isinstance(node, DroneStation) else 'depot'
                      for node in self.v]

        plt.scatter(x_values, y_values, color='black', label="Nodes")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Nodes plot")

        for node, node_type in zip(self.v[:-1], node_types[:-1]):
            plt.annotate(str(node.index), (node.location.x, node.location.y), textcoords="offset points",
                         xytext=(0, 10),
                         ha='center')

        for node, node_type in zip(self.v, node_types):
            if node_type == 'customer':
                plt.plot(node.location.x, node.location.y, 'bo', markersize=8)
            elif node_type == 'drone_station':
                plt.plot(node.location.x, node.location.y, 'r*', markersize=10)
            else:  # depot
                plt.plot(node.location.x, node.location.y, 'kD', markersize=7)

        blue_patch = plt.Line2D([0], [0], marker='o', color='w', label='Customer', markerfacecolor='blue',
                                markersize=10)
        red_patch = plt.Line2D([0], [0], marker='s', color='w', label='Drone Station', markerfacecolor='red',
                               markersize=10)
        black_patch = plt.Line2D([0], [0], marker='o', color='w', label='Depot', markerfacecolor='black', markersize=10)
        plt.legend(handles=[black_patch, red_patch, blue_patch])

        for node, node_type in zip(self.v, node_types):
            if node_type == 'drone_station':
                circle = plt.Circle((node.location.x, node.location.y), self.eps/2, color='red', fill=False, linestyle='dashed')
                plt.gca().add_patch(circle)

        if data:
            for edge in data:
                start_node = self.v[edge[0]]
                end_node = self.v[edge[1]]
                plt.arrow(start_node.location.x, start_node.location.y,
                          end_node.location.x - start_node.location.x,
                          end_node.location.y - start_node.location.y,
                          head_width=0.2, head_length=0.2, fc='blue', ec='blue')

        plt.show()

    # TODO: update plot to better a visualization
    def plotTours(self):
        tours = self.getTrucksTour()
        for tour in tours:
            tuples_tour = tourToTuple(tour)
            # print(tuples_tour)
            self.plotNodes(tuples_tour)

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
