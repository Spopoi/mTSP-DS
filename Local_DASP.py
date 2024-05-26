from itertools import chain

from core.Node import NodeType
from gurobipy import gurobipy as gp, GRB

from TourUtils import getVisitedNodesIndex, generate_sub_tours_indexes, getTrucksTour_callback, getTrucksTour, \
    varToCustomerDroneIndex, plot_dasp_tour, tourToTuple


class Local_DASP:
    def __init__(self, model, solution, d_station):
        print(f"Entro in local_DASP d: {d_station} \n con sol: {solution}")
        self.drone_to_customer = []
        self.milp_model = model
        self.tours = solution["tours"]
        self.dasp_tours = []
        self.assigned_customers = solution["assigned_customers"]
        self.dasp_solution = {"tours": [], "assigned_customers": self.assigned_customers}
        self.d_station = d_station
        self.local_tours = []
        self.V_OS = []
        self.V_OE = []
        self.O = []
        self.cost_to_start_k = []
        self.pre_dasp_nodes = []
        self.post_dasp_nodes = []

        self.V_start = None
        self.Vn = None
        self.V_end = None
        self.Vl = []
        self.Vr = []
        self.V = []

        self.set_dasp_nodes()
        # print("V = ", self.V)
        # print("V_Start = ", self.V_start)
        # print("V_End = ", self.V_end)

        self.K = range(1, len(self.dasp_tours) + 1)
        self.k = len(self.dasp_tours)

        self.V_index = None
        self.Vn_index = None
        self.Vl_index = None
        self.Vr_index = None
        self.V_OS_index = None
        self.V_OE_index = None
        self.O_index = None
        self.V_end_index = None
        self.ds_dasp_index = None

        self.set_nodes_index()

        # print("V_indexes = ", self.V_index)

        # print("V: ", self.V)
        # print("V_indexes: ", [i for i in self.V_index])
        # print("Vn_index = ", [i for i in self.Vn_index])

        # self.pre_dasp_tuples = self.get_pre_dasp_tuples()
        # print(f"pre nodes: {self.pre_dasp_nodes}, edges: {self.pre_dasp_tuples}")
        # self.post_dasp_tours = []

        self.model = None
        self.tau_tilde = None
        self.x_k_ij = None
        self.y_d_j = None
        self.a_ki = None
        self.x_k_ij_outliers = None

        self.init_model()

    def set_dasp_nodes(self):
        self.V_start = self.get_starter_nodes()
        # print("V_start =", self.V_start)
        self.Vn = self.get_ds_customers()
        # print("Vn = ", self.Vn)
        self.V_end = self.get_end_nodes()
        # print("V_end =", self.V_end)
        self.get_outlier_nodes()

        self.Vl.extend(self.V_start)
        self.Vl.extend(self.Vn)
        self.Vl.append(self.milp_model.v[self.d_station])
        self.Vl.extend(self.V_OE)

        self.Vr.extend(self.Vn)
        self.Vr.append(self.milp_model.v[self.d_station])
        self.Vr.extend(self.V_OS)
        self.Vr.extend(self.V_end)

        self.V.extend(self.V_start)
        self.V.extend(self.Vn)
        self.V.append(self.milp_model.v[self.d_station])
        self.V.extend(self.V_OS)
        self.V.extend(self.V_OE)
        self.V.extend(self.V_end)
        # print("Nodes V: ", self.V)

    def set_nodes_index(self):
        self.V_index = range(len(self.V))
        self.Vn_index = self.V_index[self.k: self.k + len(self.Vn)]

        vl_index = self.k + len(self.Vn) + 1
        self.Vl_index = self.V_index[:vl_index]
        self.Vl_index = list(
            chain(self.Vl_index, self.V_index[vl_index + len(self.V_OS): vl_index + 2 * len(self.V_OS)]))
        # print("Vl_indexes: ", self.Vl_index)

        vr_index = vl_index + len(self.V_OS)
        self.Vr_index = self.V_index[self.k: vr_index]
        self.Vr_index = list(
            chain(self.Vr_index, self.V_index[vr_index + len(self.V_OE):]))
        # print("Vr_indexes: ", self.Vr_index)

        self.V_OS_index = self.V_index[vl_index:vl_index + len(self.V_OS)]
        self.V_OE_index = self.V_index[vr_index:vr_index + len(self.V_OE)]

        self.O_index = list(zip(self.V_OS_index, self.V_OE_index))
        # print("O_index: ", self.O_index)

        self.V_end_index = self.V_index[-self.k:]
        # print("V_end_index: ", self.V_end_index)

        self.ds_dasp_index = self.k + len(self.Vn)

    def get_starter_nodes(self):
        starter_nodes = []
        ds = self.milp_model.v[self.d_station]
        # for each truck tour
        for i in range(len(self.tours)):
            cost_to_start_i = 0
            # for each node
            for j in range(1, len(self.tours[i]) - 1):
                node = self.tours[i][j]
                if node.node_distance(ds) <= self.milp_model.eps / 2:
                    starter_nodes.append(self.tours[i][j - 1])
                    self.local_tours.append(self.tours[i][j:])
                    if j-1 >= 0:
                        self.pre_dasp_nodes.append(self.tours[i][:j - 1])
                    else:
                        self.pre_dasp_nodes.append([])
                    if self.tours[i] not in self.dasp_tours:
                        self.dasp_tours.append(self.tours[i])
                    self.cost_to_start_k.append(cost_to_start_i)
                    break
                else:
                    cost_to_start_i += node.node_distance(self.tours[i][j - 1])
            if self.tours[i] not in self.dasp_tours:
                self.dasp_solution["tours"].append(self.tours[i])
                # TODO: add value
        return starter_nodes

    def get_ds_customers(self):
        Vn = []
        for tour in self.dasp_tours:
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
                if node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                    if j == len(self.dasp_tours[i]) - 1:
                        prev_node = self.dasp_tours[i][j - 1]
                        if prev_node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                            end_nodes.append(node)
                            self.post_dasp_nodes.append([])
                            break
                    else:
                        end_nodes.append(self.dasp_tours[i][j + 1])
                        self.post_dasp_nodes.append(self.dasp_tours[i][j + 2:])
                        node_index = self.local_tours[i].index(self.dasp_tours[i][j + 1])
                        self.local_tours[i] = self.local_tours[i][:node_index]
                        break
        return end_nodes

    def solve(self):
        # self.model.optimize(self.subtourelim)
        # self.model.setParam('OutputFlag', 1)
        self.model.optimize()
        tuple_tours = self.getTupleTour()
        # print("tuple_tours: ", tuple_tours)
        self.get_drone_deliveries()

        final_dasp_tours = []
        for i, tour in enumerate(tuple_tours):
            # print(f"tour {tour} pre: {self.pre_dasp_nodes[i]} post: {self.post_dasp_nodes[i]}")
            final_dasp_tours.append(self.pre_dasp_nodes[i] + [self.V[t[0]] for t in tour] + [self.V[tour[-1][1]]] + self.post_dasp_nodes[i])
        # print("dasp_modified_tours", final_dasp_tours)
        self.dasp_solution["tours"].extend(final_dasp_tours)
        # self.dasp_solution["assigned_customers"].extend(assigned_customers)
        self.dasp_solution["assigned_customers"] = self.assigned_customers
        self.dasp_solution["value"] = self.getSolution()
        # plotTours(self.model, self.V, self.milp_model.eps)
        self.plot_dasp_tours()
        return self.dasp_solution

    def getTupleTour(self):
        tours = getTrucksTour(self.model)
        tuple_tours = []
        for (i, tour) in enumerate(tours):
            # print(f"tour: {tour} \n k = {self.V_index[i]}")
            tuples_tour = tourToTuple(tour, self.V_index[i])
            tuple_tours.append(tuples_tour)
        # print(tuple_tours)
        return tuple_tours

    def plot_dasp_tours(self):
        tuple_tours = self.getTupleTour()
        for (k, tour) in enumerate(tuple_tours):
            nodes_served_by_drone = [self.V[i] for (i, _) in tour] + [self.V[tour[-1][1]]]
            complete_tour = self.pre_dasp_nodes[k] + nodes_served_by_drone + self.post_dasp_nodes[k]
            # print("ecco i tour completo dei nodi: ", complete_tour)
            plot_dasp_tour(complete_tour, self.milp_model.eps, self.V[self.ds_dasp_index], self.drone_to_customer)

    def get_drone_deliveries(self):
        customers_assigned_to_ds = []
        for decision_variable in self.model._vars:
            if "y_d_j" in decision_variable.varName:
                if decision_variable.x == 1:
                    customer = varToCustomerDroneIndex(decision_variable)
                    self.drone_to_customer.append(self.V[customer])
                    customers_assigned_to_ds.append(self.V[customer])
        self.assigned_customers.extend(customers_assigned_to_ds)
        return customers_assigned_to_ds

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

        # Constraint (custom)
        for k in self.K:
            for j in self.K:
                if j != k:
                    constraint_name_1 = f"CC_k{k}_j{j}"
                    constraint_name_2 = f"CC2_k{k}_j{j}"

                    self.model.addConstr(
                        gp.quicksum(self.x_k_ij[k, self.V_index[j - 1], i] for i in self.Vr_index) == 0,
                        name=constraint_name_1
                    )
                    self.model.addConstr(
                        gp.quicksum(self.x_k_ij[k, i, self.V_end_index[j - 1]] for i in self.Vl_index) == 0,
                        name=constraint_name_2
                    )

        self.model.addConstrs(self.x_k_ij[k, i, j] == 0 for i in self.Vl_index for j in self.Vr_index if i == j for k in self.K)

        self.model.setObjective(self.tau_tilde, sense=GRB.MINIMIZE)
        self.model.update()
        # CONSTRAINTS
        # Constraint (16)
        end_k = self.k + len(self.Vn) + 2 * len(self.O)
        # print("V = ", self.V)
        self.model.addConstrs((self.a_ki[k, end_k + k] <= self.tau_tilde for k in self.K), name="(16)")

        # Constraint (17)
        self.model.addConstrs((self.a_ki[k, self.ds_dasp_index] + gp.quicksum(
            2 * self.milp_model.t_ij_drone[self.d_station, self.V[j].index] * self.y_d_j[d, j] for j in self.Vn_index)
                               <= self.tau_tilde for k in self.K for d in self.milp_model.D), name="(17)")

        # Constraint (18)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, j] for i in self.Vl_index if i != j)
                                           for k in self.K) + gp.quicksum(self.y_d_j[d, j] for d in self.milp_model.D)
                               == 1 for j in self.Vn_index), name="(18)")

        # Constraint (19.1)
        second_step_node_indexes = []
        next_to_last_node_indexes = []
        for k in self.K:
            second_step_indexes = list(chain(self.Vn_index, [self.ds_dasp_index], self.V_OS_index, [self.V_end_index[k-1]]))
            second_step_node_indexes.append(second_step_indexes)

            next_to_last_indexes = list(chain(self.Vn_index, [self.ds_dasp_index], self.V_OE_index, [self.V_index[k-1]]))
            next_to_last_node_indexes.append(next_to_last_indexes)

        self.model.addConstrs((gp.quicksum(self.x_k_ij[k, k - 1, j] for j in second_step_node_indexes[k-1]) == 1
                               for k in self.K), name="(19.1)")

        # Constraint (19.2)
        self.model.addConstrs(
            (gp.quicksum(self.x_k_ij[k, i, self.V_end_index[k - 1]] for i in next_to_last_node_indexes[k-1]) == 1 for k in
             self.K), name="(19.2)")

        # Constraint (20.1)
        self.model.addConstr((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, self.ds_dasp_index] for i in self.Vl_index
                                                      if i != self.ds_dasp_index) for k in self.K) == 1), name="(20.1)")

        # Constraint (20.2)
        self.model.addConstr((gp.quicksum(gp.quicksum(self.x_k_ij[k, self.ds_dasp_index, j] for j in self.Vr_index
                                                      if j != self.ds_dasp_index) for k in self.K) == 1), name="(20.2)")

        # Constraint (custom: ds_flow_balance)
        self.model.addConstrs(((gp.quicksum(self.x_k_ij[k, i, self.ds_dasp_index] for i in self.Vl_index
                                            if i != self.ds_dasp_index) == gp.quicksum(self.x_k_ij[k, self.ds_dasp_index, j]
                                                                                       for j in self.Vr_index if j != self.ds_dasp_index)) for k in self.K), name = "ds_flow_balance")

        # Constraint (21)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, os] for i in self.Vl_index if i != oe)
                                           for k in self.K) == 1 for (os, oe) in self.O_index), name="(21)")
        # Constraint (22)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, oe, j] for j in self.Vr_index if j != os)
                                           for k in self.K) == 1 for (os, oe) in self.O_index), name="(22)")

        # Constraint (23)
        self.model.addConstrs((gp.quicksum(self.x_k_ij[k, i, os] for i in self.Vl_index) - self.x_k_ij[k, os, oe] == 0
                               for k in self.K for (os, oe) in self.O_index), name="(23)")

        # Constraint (24)
        self.model.addConstrs(
            (gp.quicksum(self.x_k_ij[k, i, h] for i in self.Vl_index if i != h) -
             gp.quicksum(self.x_k_ij[k, h, j] for j in self.Vr_index if j != h) == 0
             for k in self.K for h in self.Vn_index), name="(24)")

        # Constraint (25)
        self.model.addConstrs((self.cost_to_start_k[k - 1] == self.a_ki[k, k - 1] for k in self.K), name="(25)")

        # Constraint (26)
        M = 10000
        earliest_arrival_time_set = list(chain(self.Vn_index, [self.ds_dasp_index], self.V_OS_index))
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
