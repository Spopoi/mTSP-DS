from itertools import chain

from Node import NodeType
from gurobipy import gurobipy as gp, GRB

from TourUtils import getVisitedNodesIndex, generate_sub_tours_indexes, getTrucksTour_callback, plotNodes, plotTours, \
    getTrucksTour, getTupleTour, varToCustomerDroneIndex


class Local_DASP:
    def __init__(self, model, solution, d_station):
        self.milp_model = model
        self.tours = solution["tours"]
        self.dasp_tours = []
        self.assigned_customers = solution["assigned_customers"]
        self.d_station = d_station
        # TODO: check local_tours, may not be present ds in the local tour
        self.local_tours = []
        self.V_OS = []
        self.V_OE = []
        self.O = []
        self.cost_to_start_k = []
        self.V_start = self.get_starter_nodes()
        print("V_start =", self.V_start)
        self.Vn = self.get_ds_customers()
        # print("Vn = ", self.Vn)
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

        self.V_index = range(len(self.V))
        print("V_indexes: ", [self.V_index])
        print("V: ", self.V)

        self.K = range(1, len(self.dasp_tours) + 1)
        self.k = len(self.dasp_tours)

        self.Vn_index = self.V_index[self.k: self.k + len(self.Vn)]
        print("Vn_index = ", self.Vn_index)

        vl_index = self.k + len(self.Vn) + 1
        self.Vl_index = self.V_index[:vl_index]
        self.Vl_index = list(
            chain(self.Vl_index, self.V_index[vl_index + len(self.V_OS): vl_index + 2 * len(self.V_OS)]))
        print("Vl_indexes: ", self.Vl_index)

        vr_index = vl_index + len(self.V_OS)
        self.Vr_index = self.V_index[self.k: vr_index]
        self.Vr_index = list(
            chain(self.Vr_index, self.V_index[vr_index + len(self.V_OE):]))
        print("Vr_indexes: ", self.Vr_index)

        self.V_OS_index = self.V_index[vl_index:vl_index + len(self.V_OS)]
        self.V_OE_index = self.V_index[vr_index:vr_index + len(self.V_OE)]

        self.O_index = list(zip(self.V_OS_index, self.V_OE_index))
        print("O_index: ", self.O_index)

        self.V_end_index = self.V_index[-self.k:]
        print("V_end_index: ", self.V_end_index)

        self.model = None
        self.tau_tilde = None
        self.x_k_ij = None
        self.y_d_j = None
        self.a_ki = None
        self.x_k_ij_outliers = None
        self.init_model()

    def get_starter_nodes(self):
        # print("entro in starter nodes")
        starter_nodes = []
        ds = self.milp_model.v[self.d_station]
        # for each truck tour
        for i in range(len(self.tours)):
            # self.local_tours.append([])
            # print("truck ", i)
            cost_to_start_i = 0
            # for each node
            for j in range(1, len(self.tours[i]) - 1):
                node = self.tours[i][j]
                # print("node(i,j) ", node)
                if node.node_distance(ds) <= self.milp_model.eps / 2:
                    starter_nodes.append(self.tours[i][j - 1])
                    self.local_tours.append(self.tours[i][j:])
                    # print("Sono in start node e i local tours sono: ", self.local_tours)
                    if self.tours[i] not in self.dasp_tours:
                        self.dasp_tours.append(self.tours[i])
                    self.cost_to_start_k.append(cost_to_start_i)
                    break
                else:
                    cost_to_start_i += node.node_distance(self.tours[i][j - 1])
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
        for i in range(len(self.dasp_tours)):
            for j in range(len(self.dasp_tours[i]) - 1, -1, -1):
                node = self.dasp_tours[i][j]
                print(f"{(i, j)} iteration, node: {node}")
                if node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                    if j == len(self.dasp_tours[i]) - 1:
                        prev_node = self.dasp_tours[i][j - 1]
                        if prev_node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                            end_nodes.append(node)
                            print(f"local tours {i}", self.local_tours[i])
                            break
                    else:
                        # print("non è l'ultimo nodo..")
                        end_nodes.append(self.dasp_tours[i][j + 1])
                        node_index = self.local_tours[i].index(self.dasp_tours[i][j + 1])
                        self.local_tours[i] = self.local_tours[i][:node_index]
                        print(f"local tours {i}", self.local_tours[i])
                        break
                    print(f"local tours {i}", self.local_tours[i])
                    # break
        print("end_nodes: ", end_nodes)
        return end_nodes

    def solve(self):
        print("start", self.V_start)
        print("end", self.V_end)
        print("Outliers= ", self.O)
        # self.model.optimize(self.subtourelim)
        self.model.optimize()
        print("dasp_solution = ", self.getSolution())
        # tuple_tours = getTupleTour(self.model)
        assigned_customers = self.get_assigned_customers()
        solution = {"tours": [], "assigned_customers": []}
        tours = getTrucksTour(self.model)
        solution[tours].extend(tours)
        solution[assigned_customers].extend(assigned_customers)
        return solution

    def get_assigned_customers(self):
        tuple_tours = getTupleTour(self.model)
        assigned_customers = []
        for tour in tuple_tours:
            for tour_tuple in tour:
                node = self.V[tour_tuple[0]]
                if node.node_type == NodeType.CUSTOMER:
                    assigned_customers.append(node)
        customers_served_by_drones = self.get_drone_customers()
        assigned_customers.extend(customers_served_by_drones)
        return assigned_customers

    def get_drone_customers(self):
        customers_assigned_to_ds = []
        for decision_variable in self.model._vars:
            if "y_d_j" in decision_variable.varName:
                if decision_variable.x == 1:
                    customer = varToCustomerDroneIndex(decision_variable)
                    customers_assigned_to_ds.append(customer)
        return customers_assigned_to_ds

    def plot_tours(self):
        plotTours(self.model, self.V, self.milp_model.eps)

    def getSolution(self):
        return self.model.ObjVal

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
                        # self.traversal_cost[(node, node)] = 0
                    else:
                        oe_node = self.get_end_outlier(i, j + 1)
                    self.V_OE.append(oe_node)
                    j = self.local_tours[i].index(oe_node) + 1
                    self.O.append((node, oe_node))
                else:
                    j += 1

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

        x_k_ij_indexes = [(k, i, j) for k in self.K for i in self.Vl_index for j in self.Vr_index]
        x_k_ij_indexes += [(k, i, j) for k in self.K for (i, j) in self.O_index]

        self.x_k_ij = self.model.addVars(x_k_ij_indexes, vtype=GRB.BINARY, name="x_k_ij")

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
        end_k = self.k + len(self.Vn) + 2 * len(self.O)
        # print("V = ", self.V)
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

        self.model.addConstrs((gp.quicksum(self.x_k_ij[k, k - 1, j] for j in second_step_node_indexes) == 1
                               for k in self.K), name="(19.1)")

        # Constraint (19.2)
        next_to_last_node_indexes = list(chain(self.Vn_index, [ds_dasp_index], self.V_OE_index, self.V_index[:self.k]))
        self.model.addConstrs(
            (gp.quicksum(self.x_k_ij[k, i, self.V_end_index[k - 1]] for i in next_to_last_node_indexes) == 1 for k in
             self.K), name="(19.2)")

        # Constraint (20.1)
        self.model.addConstr((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, ds_dasp_index] for i in self.Vl_index
                                                      if i != ds_dasp_index) for k in self.K) == 1), name="(20.1)")

        # Constraint (20.2)
        self.model.addConstr((gp.quicksum(gp.quicksum(self.x_k_ij[k, ds_dasp_index, j] for j in self.Vr_index
                                                      if j != ds_dasp_index) for k in self.K) == 1), name="(20.2)")

        # Constraint (21)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, os] for i in self.Vl_index if i != oe)
                                           for k in self.K) == 1 for (os, oe) in self.O_index), name="(21)")
        # Constraint (22)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, oe, j] for j in self.Vr_index if j != os)
                                           for k in self.K) == 1 for (os, oe) in self.O_index), name="(22)")


        self.model.addConstrs((gp.quicksum(self.x_k_ij[k, i, os] -
                                           self.x_k_ij[k, os, oe] for i in self.Vl_index) == 0
                               for k in self.K for (os, oe) in self.O_index), name="(23)")

        # Constraint (24)
        self.model.addConstrs(
            (gp.quicksum(self.x_k_ij[k, i, h] for i in self.Vl_index if i != h) -
             gp.quicksum(self.x_k_ij[k, h, j] for j in self.Vr_index if j != h) == 0
             for k in self.K for h in self.Vn_index), name="(24)")

        # Constraint (25)
        self.model.addConstrs((self.cost_to_start_k[k - 1] == self.a_ki[k, k - 1] for k in self.K), name="(25)")

        # Constraint (26)
        M = 1000
        earliest_arrival_time_set = list(chain(self.Vn_index, [ds_dasp_index], self.V_OS_index))
        self.model.addConstrs((M * (self.x_k_ij[k, i, j] - 1) + self.a_ki[k, i]
                               + self.milp_model.t_ij[self.V[i].index, self.V[j].index] <= self.a_ki[k, j]
                               for k in self.K for i in self.Vl_index for j in earliest_arrival_time_set if i != j)
                              , name="(26)")

        # Constraint (27)
        self.model.addConstrs((M * (self.x_k_ij[k, os, oe] - 1) + self.a_ki[k, os]
                               + self.traversal_cost(os, oe) <= self.a_ki[k, oe] for k in self.K
                               for (os, oe) in self.O_index), name="(27)")

        cost_after_end_k = self.cost_after_end_k(end_k)
        # Constraint (28)
        self.model.addConstrs((M * (self.x_k_ij[k, i, end_k + k] - 1) + self.a_ki[k, i] + self.milp_model.t_ij[
            self.V[i].index, self.V[end_k + k].index]
                               + cost_after_end_k[k - 1] <= self.a_ki[k, end_k + k] for k in self.K
                               for i in self.Vl_index), name="(28)")

        self.model.update()
        self.model.write("modello_matheuristic.lp")
        self.model._edges = self.x_k_ij
        self.model._vars = self.model.getVars()
        self.model.setParam('OutputFlag', 0)
        self.model.Params.lazyConstraints = 1

    def subtourelim(self, model, where):
        if where == GRB.Callback.MIPSOL:
            tours = getTrucksTour_callback(model)
            truck_index = 1
            x_k_ij = model._edges
            for truck_tour in tours:
                node_indexes = getVisitedNodesIndex(truck_tour)
                # print("node_indexes", node_indexes)
                sub_tours_indexes = generate_sub_tours_indexes(node_indexes[1:-1])
                # print("sub_tours_indexes", sub_tours_indexes)
                # Constraint (13)
                for S in sub_tours_indexes:
                    model.cbLazy(
                        gp.quicksum(gp.quicksum(x_k_ij[truck_index, i, j] for j in S if i != j) for i in S)
                        <= len(S) - 1)
                    model.update()
                truck_index += 1

    def traversal_cost(self, os, oe):
        if self.V[os] == self.V[oe]:
            return 0
        outliers = (self.V[os], self.V[oe])
        traversal_cost = 0
        for i in range(len(self.dasp_tours)):
            tour = self.dasp_tours[i]
            for j in range(len(tour)):
                node = tour[j]
                if node == outliers[0]:
                    traversal_cost += node.node_distance(tour[j + 1])
                elif node == outliers[1]:
                    break
        return traversal_cost

    def cost_after_end_k(self, end_k):
        cost_after_end_k = []
        for k in self.K:
            end_node_k = self.V[end_k + k]
            if end_node_k.node_type == NodeType.DEPOT:
                cost_after_end_k.append(0)
                continue
            tour = self.dasp_tours[k - 1]
            end_node_dasp_tour_index = tour.index(end_node_k)
            cost = 0
            for (i, node) in enumerate(tour[end_node_dasp_tour_index:-1]):
                cost += node.node_distance(tour[end_node_dasp_tour_index + i + 1])
            cost_after_end_k.append(cost)
        return cost_after_end_k
