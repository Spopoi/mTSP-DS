import numpy as np

from Node import NodeType
from gurobipy import gurobipy as gp, GRB


class Local_DASP:
    def __init__(self, model, solution, d_station):
        self.milp_model = model
        self.tours = solution["tours"]
        # print(self.tours)
        self.dasp_tours = []
        # self.dasp_tours = self.get_local_dasp_tours(self.tours)
        self.assigned_customers = solution["assigned_customers"]
        self.d_station = d_station
        # print(d_station)
        # TODO: check local_tours, may not be present ds in the local tour
        self.local_tours = []
        self.V_OS = []
        self.V_OE = []
        self.O = []
        self.V_start = self.get_starter_nodes()
        self.Vn = self.get_ds_customers()
        self.Vn_index = [node.index for node in self.Vn]
        print("Nodes: ", self.Vn_index)
        self.V_end = self.get_end_nodes()
        self.get_outlier_nodes()

        self.Vl = []
        self.Vl.extend(self.V_start)
        self.Vl.extend(self.Vn)
        self.Vl.append(self.milp_model.v[d_station])
        self.Vl.extend(self.V_OE)
        print("Vl = ", self.Vl)

        self.Vr = []
        self.Vr.extend(self.Vn)
        self.Vr.append(self.milp_model.v[d_station])
        self.Vr.extend(self.V_OS)
        self.Vr.extend(self.V_end)
        print("Vr = ", self.Vr)

        self.V = []
        self.V.extend(self.V_start)
        self.V.extend(self.Vn)
        self.V.append(self.milp_model.v[d_station])
        self.V.extend(self.V_OS)
        self.V.extend(self.V_OE)
        self.V.extend(self.V_end)

        # TODO: repetitions allowed?
        self.V_index = [node.index for node in self.V]
        self.Vl_index = [node.index for node in self.Vl]
        self.Vr_index = [node.index for node in self.Vr]

        self.model = None
        self.tau_tilde = None
        self.x_k_ij = None
        self.y_d_j = None
        self.a_ki = None
        self.init_model()

    def get_starter_nodes(self):
        starter_nodes = []
        # for each truck tour
        for i in range(len(self.tours)):
            self.local_tours.append([])
            # for each node
            for j in range(1, len(self.tours[i])):
                node = self.tours[i][j]
                # print(f"Station= {d_station} location = {self.v[d_station].location}")
                # print(f"Node {node.index} Location = {node.location}")
                # print("distance=", node.node_distance(self.v[d_station]))
                # print("eps=", self.eps / 2)
                # print("RESULT= ", node.node_distance(self.v[d_station]) <= self.eps / 2)
                if node.node_type == NodeType.CUSTOMER and node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                    # starter_nodes.append((i, self.tours[i][j - 1]))
                    starter_nodes.append(self.tours[i][j - 1])
                    self.local_tours[i] = self.tours[i][j:]
                    if self.tours[i] not in self.dasp_tours:
                        self.dasp_tours.append(self.tours[i])
                    break
        # print(self.dasp_tours)
        return starter_nodes

    def get_ds_customers(self):
        Vn = []
        for tour in self.tours:
            for node in tour:
                if (node.node_type == NodeType.CUSTOMER and node.node_distance(
                        self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2
                        and node not in self.assigned_customers):
                    Vn.append(node)
        return Vn

    def get_end_nodes(self):
        end_nodes = []
        print("starting end nodes, nodes: ", self.tours)
        # for each truck tour
        for i in range(len(self.tours)):
            # for each node
            for j in range(len(self.tours[i])-1, -1, -1):
                node = self.tours[i][j]
                print(len(self.tours[i]))
                print(j)
                if node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                    # end_nodes.append((i, self.tours[i][j + 1]))
                    if j == len(self.tours[i]) - 1:
                        end_nodes.append(self.tours[i][j])
                    else:
                        end_nodes.append(self.tours[i][j + 1])

                    # Rimuovi tutti gli elementi dopo 'node' in 'self.local_tours[i]'
                    node_index = self.local_tours[i].index(node)
                    self.local_tours[i] = self.local_tours[i][:node_index + 1]
                    break
        # print("end nodes: ", end_nodes)
        return end_nodes

    def solve(self):
        print("start", self.V_start)
        print("end", self.V_end)
        print("tour_nodes", self.Vn)
        # print("Local tours= ", self.local_tours)
        print("Outliers= ", self.O)

    def get_outlier_nodes(self):
        for i in range(len(self.local_tours)):
            j = 0
            while j < len(self.local_tours[i]):
                node = self.local_tours[i][j]
                if node.node_type == NodeType.CUSTOMER and node.node_distance(
                        self.milp_model.v[self.d_station]) > self.milp_model.eps / 2:
                    self.V_OS.append(node)
                    if len(self.local_tours[i]) == 1:
                        oe_node = node
                    else:
                        oe_node = self.get_end_outlier(i, j + 1)
                    self.V_OE.append(oe_node)
                    # TODO: verify it works
                    j = self.local_tours[i].index(oe_node) + 1
                    self.O.append((node, oe_node))
                else:
                    j += 1  # Se non è un nodo outlier, passa al prossimo

    def get_end_outlier(self, tour_k, node_index):
        # for each node
        for j in range(node_index, len(self.local_tours[tour_k])):
            node = self.local_tours[tour_k][j]
            if node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                return self.local_tours[tour_k][j - 1]

    def init_model(self):
        self.model = gp.Model("Local_DASP")

        # DECISION VARIABLES
        # Makespan
        self.tau_tilde = self.model.addVar(vtype=GRB.CONTINUOUS, name="tau")

        # truck k traverse edge (i,j)
        # TODO: add (os,oe) pairs
        # self.x_k_ij = self.model.addVars([(k, i, j) for k in self.milp_model.K for i in self.Vl_index
        #                                   for j in self.Vr_index], vtype=GRB.BINARY, name="x_k_ij")
        #
        # # time truck k arrives at node i
        # self.a_ki = self.model.addVars([(i, j) for i in self.milp_model.K for j in self.V_index],
        #                                vtype=GRB.CONTINUOUS, name="a_ki")
        #
        # # customer j served by drone d from drone station s
        # self.y_d_j = self.model.addVars([(d, j) for d in self.milp_model.D for j in self.Vn_index],
        #                                 vtype=GRB.BINARY, name="y_d_j")
        # self.model.update()
        # self.model.setObjective(self.tau_tilde, sense=GRB.MINIMIZE)
