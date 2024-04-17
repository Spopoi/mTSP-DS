from itertools import chain

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
        # self.Vn_index = [node.index for node in self.Vn]
        print("Vn = ", self.Vn)
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

        # self.V_index = [node.index for node in self.V]
        # self.Vl_index = [node.index for node in self.Vl]
        # #print("Vl_indexes: ", self.Vl_index)
        # self.Vr_index = [node.index for node in self.Vr]

        self.V_index = range(len(self.V))
        print("V_indexes: ", self.V_index)
        print("V: ", self.V)

        # self.Vl_index = range(len(self.Vl))
        # print("Vl_indexes: ", self.Vl_index)
        # self.Vr_index = range(len(self.Vr))
        # self.K = range(1, len(self.dasp_tours)+1)
        self.K = range(1, len(self.dasp_tours) + 1)
        self.k = len(self.dasp_tours)

        self.Vn_index = self.V_index[self.k: self.k + len(self.Vn)]
        print("Vn_index = ", self.Vn_index)

        vl_index = self.k + len(self.Vn) + 1
        self.Vl_index = self.V_index[:vl_index]
        self.Vl_index = list(chain(self.Vl_index, self.V_index[vl_index + len(self.V_OS): vl_index + 2 * len(self.V_OS)]))
        print("Vl_indexes: ", self.Vl_index)

        vr_index = vl_index + len(self.V_OS)
        self.Vr_index = self.V_index[self.k: vr_index]
        self.Vr_index = list(
            chain(self.Vr_index, self.V_index[vr_index+len(self.V_OE):]))
        print("Vr_indexes: ", self.Vr_index)

        # self.O_index = [(i, j) for i in self.V_index[vl_index:vl_index+len(self.V_OS)]
        # for j in self.V_index[vr_index:vr_index+len(self.V_OE)]]

        self.V_OS_index = self.V_index[vl_index:vl_index+len(self.V_OS)]
        self.V_OE_index = self.V_index[vr_index:vr_index+len(self.V_OE)]

        self.O_index = list(zip(self.V_OS_index, self.V_OE_index))
        print("O_index: ", self.O_index)
        # self.k = len(self.dasp_tours)

        self.V_end_index = self.V_index[-self.k:]

        self.model = None
        self.tau_tilde = None
        self.x_k_ij = None
        self.y_d_j = None
        self.a_ki = None
        self.x_k_ij_outliers = None
        self.init_model()

    # TODO: check case in which the only node in ds range is depot
    def get_starter_nodes(self):
        starter_nodes = []
        # for each truck tour
        for i in range(len(self.tours)):
            self.local_tours.append([])
            # for each node
            for j in range(1, len(self.tours[i])):
                node = self.tours[i][j]
                # TODO: check, is considered also the case in which the only node in range of ds is ds itself
                if node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                    # starter_nodes.append((i, self.tours[i][j - 1]))
                    starter_nodes.append(self.tours[i][j - 1])
                    self.local_tours[i] = self.tours[i][j:]
                    if self.tours[i] not in self.dasp_tours:
                        self.dasp_tours.append(self.tours[i])
                    break
        print("dasp_tours: ", self.dasp_tours)
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
        # for each truck tour
        for i in range(len(self.tours)):
            # for each node
            for j in range(len(self.tours[i])-1, -1, -1):
                node = self.tours[i][j]
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
        # print("tour_nodes", self.Vn)
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
        self.tau_tilde = self.model.addVar(vtype=GRB.CONTINUOUS, name="tau_tilde")

        # truck k traverse edge (i,j)
        self.x_k_ij = self.model.addVars([(k, i, j) for k in self.K for i in self.Vl_index
                                          for j in self.Vr_index], vtype=GRB.BINARY, name="x_k_ij")

        self.x_k_ij_outliers = self.model.addVars([(k, i, j) for k in self.K for (i, j) in self.O_index],
                                                  vtype=GRB.BINARY, name="x_k_ij_outliers")

        # time truck k arrives at node i
        self.a_ki = self.model.addVars([(i, j) for i in self.K for j in self.V_index],
                                       vtype=GRB.CONTINUOUS, name="a_ki")

        # customer j served by drone d from drone station s
        self.y_d_j = self.model.addVars([(d, j) for d in self.milp_model.D for j in self.Vn_index],
                                        vtype=GRB.BINARY, name="y_d_j")
        self.model.update()
        self.model.setObjective(self.tau_tilde, sense=GRB.MINIMIZE)

        # CONSTRAINTS
        # Constraint (16)
        end_k = self.k + len(self.Vn) + 2*len(self.O)
        print("V = ", self.V)
        self.model.addConstrs((self.a_ki[k, end_k + k] <= self.tau_tilde for k in self.K), name="(16)")

        # Constraint (17)
        ds_dasp_index = self.k + len(self.Vn)
        self.model.addConstrs((self.a_ki[k, ds_dasp_index] + gp.quicksum(
            2 * self.milp_model.t_ij_drone[self.d_station, self.V[j].index] * self.y_d_j[d, j] for j in self.Vn_index)
                               <= self.tau_tilde for k in self.K for d in self.milp_model.D), name="(17)")

        # Constraint (18)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, j] for i in self.Vl_index if i != j)
                                           for k in self.K) + gp.quicksum(self.y_d_j[d, j] for d in self.milp_model.D)
                               == 1 for j in self.Vn_index), name="(18)")

        # Constraint (19.1)
        second_step_node_indexes = list(chain(self.Vn_index, [ds_dasp_index], self.V_OS_index, self.V_end_index))
        print("Second_step: ", second_step_node_indexes)

        self.model.addConstrs((gp.quicksum(self.x_k_ij[k, k-1, j] for j in second_step_node_indexes) == 1
                               for k in self.K), name="(19.1)")

        # Constraint (19.2)
        # self.model.addConstrs(
        #     (gp.quicksum(self.x_k_ij[k, i, 1 + self.n + self.m] for i in self.Vn_index) == 1 for k in self.K), name="(19.2)")

        self.model.update()
        self.model.write("modello_matheuristic.lp")

