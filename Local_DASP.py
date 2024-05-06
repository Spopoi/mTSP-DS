from itertools import chain

from core.Node import NodeType
from gurobipy import gurobipy as gp, GRB

from TourUtils import getVisitedNodesIndex, generate_sub_tours_indexes, getTrucksTour_callback, getTrucksTour, \
    varToCustomerDroneIndex, varToTupleIndex, get_customer_drone_edges, plotTours, plotNodes


class Local_DASP:
    def __init__(self, model, solution, d_station):
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
        self.pre_dasp_tours = []
        self.post_dasp_tours = []
        self.V_start = self.get_starter_nodes()
        print("V_start =", self.V_start)
        self.Vn = self.get_ds_customers()
        # print("Vn = ", self.Vn)
        self.V_end = self.get_end_nodes()
        print("V_end =", self.V_end)
        self.get_outlier_nodes()

        self.Vl = []
        self.Vl.extend(self.V_start)
        self.Vl.extend(self.Vn)
        self.Vl.append(self.milp_model.v[d_station])
        self.Vl.extend(self.V_OE)
        # print("Vl = ", self.Vl)

        self.Vr = []
        self.Vr.extend(self.Vn)
        self.Vr.append(self.milp_model.v[d_station])
        self.Vr.extend(self.V_OS)
        self.Vr.extend(self.V_end)
        # print("Vr = ", self.Vr)

        self.V = []
        self.V.extend(self.V_start)
        self.V.extend(self.Vn)
        self.V.append(self.milp_model.v[d_station])
        self.V.extend(self.V_OS)
        self.V.extend(self.V_OE)
        self.V.extend(self.V_end)

        self.V_index = range(len(self.V))
        print("V_indexes: ", [i for i in self.V_index])
        print("V: ", self.V)

        self.K = range(1, len(self.dasp_tours) + 1)
        self.k = len(self.dasp_tours)

        self.Vn_index = self.V_index[self.k: self.k + len(self.Vn)]
        print("Vn_index = ", [i for i in self.Vn_index])

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
        print("O_index: ", self.O_index)

        self.V_end_index = self.V_index[-self.k:]
        # print("V_end_index: ", self.V_end_index)

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
                    if j-1 >= 0:
                        self.pre_dasp_tours.append(self.tours[i][:j-1])
                    else:
                        self.pre_dasp_tours.append([])
                    # print(f"local_tours : {self.local_tours}, pre_dasp_tours: {self.pre_dasp_tours}")
                    # print("Sono in start node e i local tours sono: ", self.local_tours)
                    if self.tours[i] not in self.dasp_tours:
                        self.dasp_tours.append(self.tours[i])
                    self.cost_to_start_k.append(cost_to_start_i)
                    break
                else:
                    cost_to_start_i += node.node_distance(self.tours[i][j - 1])
                    # print("cost_to_start: ", cost_to_start_i)
            if self.tours[i] not in self.dasp_tours:
                self.dasp_solution["tours"].append(self.tours[i])

        # print(f"dasp_tours: {self.dasp_tours}")
        # print(f"solution: ", self.dasp_solution["tours"])
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
                # print(f"{(i, j)} iteration, node: {node}")
                if node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                    if j == len(self.dasp_tours[i]) - 1:
                        prev_node = self.dasp_tours[i][j - 1]
                        if prev_node.node_distance(self.milp_model.v[self.d_station]) <= self.milp_model.eps / 2:
                            end_nodes.append(node)
                            self.post_dasp_tours.append([])
                            # print(f"local tours {i}", self.local_tours[i])
                            break
                    else:
                        # print("non è l'ultimo nodo..")
                        end_nodes.append(self.dasp_tours[i][j + 1])
                        self.post_dasp_tours.append(self.dasp_tours[i][j + 2:])
                        node_index = self.local_tours[i].index(self.dasp_tours[i][j + 1])
                        self.local_tours[i] = self.local_tours[i][:node_index]
                        # print(f"local tours {i}", self.local_tours[i])
                        break
                    # print(f"local tours {i}", self.local_tours[i])
                    # break
        # print("end_nodes: ", end_nodes)
        # print("self.post_dasp_tours: ", self.post_dasp_tours)
        return end_nodes

    def solve(self):
        # self.model.optimize(self.subtourelim)
        self.model.optimize()
        # print("dasp_solution = ", self.getSolution())
        # print("drone assignment = ", self.get_drone_customers())
        # tuple_tours = getTupleTour(self.model)
        tuple_tours = self.getTupleTour()
        print("tuple_tours: ", tuple_tours)
        self.get_assigned_customers(tuple_tours)

        # tours = getTrucksTour(self.model)
        final_dasp_tours = []
        for i, tour in enumerate(tuple_tours):
            print(f"tour {tour} pre: {self.pre_dasp_tours[i]} post: {self.post_dasp_tours[i]}")
            final_dasp_tours.append(self.pre_dasp_tours[i] + [self.V[t[0]] for t in tour] + [self.V[tour[-1][1]]] + self.post_dasp_tours[i])
        print("dasp_modified_tours", final_dasp_tours)
        self.dasp_solution["tours"].extend(final_dasp_tours)
        # self.dasp_solution["assigned_customers"].extend(assigned_customers)
        self.dasp_solution["assigned_customers"] = self.assigned_customers
        #plotTours(self.model, self.V, self.milp_model.eps)
        self.plotTours()
        return self.dasp_solution

    def tourToTuple(self, tour, k):
        ordered_tuple_tour = []
        tuple_tour = [varToTupleIndex(var) for var in tour]
        # print("le tuple sono: ", tuple_tour)
        # tuple_tour_node_indexes = [(self.V[edge[0]].index, self.V[edge[1]].index) for edge in tuple_tour]
        # print("ECCOLO IL TUPLETOUR in localDasp: ", tuple_tour_node_indexes)
        # filtered_tuple = list(filter(lambda x: x[0] == self.V_start[k].index, tuple_tour_node_indexes))[0]
        filtered_tuple = list(filter(lambda x: x[0] == self.V_index[k], tuple_tour))[0]
        # print("la prima tappa è: ", filtered_tuple)
        ordered_tuple_tour.append(filtered_tuple)
        # tuple_tour_node_indexes.remove(filtered_tuple)
        tuple_tour.remove(filtered_tuple)
        for i in range(len(tuple_tour)):
            nextTuple = list(filter(lambda x: x[0] == filtered_tuple[1], tuple_tour))[0]
            # nextTuple = list(filter(lambda x: x[0] == filtered_tuple[1], tuple_tour_node_indexes))[0]
            ordered_tuple_tour.append(nextTuple)
            # tuple_tour_node_indexes.remove(nextTuple)
            tuple_tour.remove(nextTuple)
            filtered_tuple = nextTuple
        return ordered_tuple_tour

    def getTupleTour(self):
        tours = getTrucksTour(self.model)
        tuple_tours = []
        # print("update tours: ", tours)
        for (i, tour) in enumerate(tours):
            # print(f"Sto per prendere le tuple di {tour} del camion {i}")
            tuples_tour = self.tourToTuple(tour, i)
            # print("tuple_tour: ", tuples_tour)
            tuple_tours.append(tuples_tour)
        print(tuple_tours)
        return tuple_tours

    def plotTours(self):
        tuple_tours = self.getTupleTour()
        drone_deliveries = get_customer_drone_edges(self.model)
        drone_deliveries_from_s = None
        for tour in tuple_tours:
            for i, _ in tour:
                if self.V[i].node_type == NodeType.DRONE_STATION:
                    drone_deliveries_from_s = list(filter(lambda x: x[0] == i, drone_deliveries))
            plotNodes(self.V, self.milp_model.eps, tour, drone_deliveries_from_s)

    def get_assigned_customers(self, tuple_tours):
        # print("entro in get_assigned_customers")
        # tuple_tours = self.getTupleTour()
        # for tour in tuple_tours:
        #     for tour_tuple in tour:
        #         node = self.V[tour_tuple[0]]
        #         # print("node = ", node)
        #         if node.node_type == NodeType.CUSTOMER and node not in self.assigned_customers:
        #             self.assigned_customers.append(node)
        customers_served_by_drones = self.get_drone_customers()
        self.assigned_customers.extend(customers_served_by_drones)
        # return assigned_customers

    # def get_assigned_customers(self, tuple_tours):
    #     # print("entro in get_assigned_customers")
    #     # tuple_tours = self.getTupleTour()
    #     assigned_customers = []
    #     for tour in tuple_tours:
    #         for tour_tuple in tour:
    #             node = self.V[tour_tuple[0]]
    #             if node.node_type == NodeType.CUSTOMER and node not in assigned_customers:
    #                 assigned_customers.append(node)
    #     customers_served_by_drones = self.get_drone_customers()
    #     assigned_customers.extend(customers_served_by_drones)
    #     return assigned_customers

    def get_drone_customers(self):
        customers_assigned_to_ds = []
        for decision_variable in self.model._vars:
            if "y_d_j" in decision_variable.varName:
                if decision_variable.x == 1:
                    customer = varToCustomerDroneIndex(decision_variable)
                    customers_assigned_to_ds.append(self.V[customer])
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

        # for (k, i, j) in x_k_ij_indexes:
        #     print(f"(k,i,j): {(k,i,j)}")z

        self.x_k_ij = self.model.addVars(x_k_ij_indexes, vtype=GRB.BINARY, name="x_k_ij")

        # time truck k arrives at node i
        self.a_ki = self.model.addVars([(i, j) for i in self.K for j in self.V_index],
                                       vtype=GRB.CONTINUOUS, name="a_ki")
        # customer j served by drone d from drone station s
        self.y_d_j = self.model.addVars([(d, j) for d in self.milp_model.D for j in self.Vn_index],
                                        vtype=GRB.BINARY, name="y_d_j")

        # self.model.update()
        # for (k, i, j) in x_k_ij_indexes:
        #     # Imposta il valore Start a 0
        #     self.x_k_ij[k, i, j].Start = 0
        # self.model.update()
        # self.set_start_variables()
        self.model.update()
        for (k, i, j) in x_k_ij_indexes:
            var = self.x_k_ij[k, i, j]
            if var.Start == 1:
                print(f"{(k, i, j)} : Start={var.Start}, VarName={var.VarName}")

        for k in self.K:
            for j in self.K:
                if j != k:
                    self.model.addConstr((gp.quicksum(self.x_k_ij[k, self.V_index[j - 1], i] for i in self.Vr_index) == 0), name="CC")
                    self.model.addConstr((gp.quicksum(self.x_k_ij[k, i, self.V_end_index[j - 1]] for i in self.Vl_index) == 0), name="CC2")


        # self.x_k_ij[1, 0, 2].Start = 1
        self.model.setObjective(self.tau_tilde, sense=GRB.MINIMIZE)
        self.model.update()
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
        # second_step_node_indexes = list(chain(self.Vn_index, [ds_dasp_index], self.V_OS_index, self.V_end_index))
        second_step_node_indexes = []
        next_to_last_node_indexes = []
        for k in self.K:
            second_step_indexes = list(chain(self.Vn_index, [ds_dasp_index], self.V_OS_index, [self.V_end_index[k-1]]))
            second_step_node_indexes.append(second_step_indexes)

            next_to_last_indexes = list(chain(self.Vn_index, [ds_dasp_index], self.V_OE_index, [self.V_index[k-1]]))
            # print(next_to_last_indexes)
            next_to_last_node_indexes.append(next_to_last_indexes)

        self.model.addConstrs((gp.quicksum(self.x_k_ij[k, k - 1, j] for j in second_step_node_indexes[k-1]) == 1
                               for k in self.K), name="(19.1)")

        # Constraint (19.2)
        # next_to_last_node_indexes = list(chain(self.Vn_index, [ds_dasp_index], self.V_OE_index, self.V_index[:self.k]))
        self.model.addConstrs(
            (gp.quicksum(self.x_k_ij[k, i, self.V_end_index[k - 1]] for i in next_to_last_node_indexes[k-1]) == 1 for k in
             self.K), name="(19.2)")

        # Constraint (20.1)
        self.model.addConstr((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, ds_dasp_index] for i in self.Vl_index
                                                      if i != ds_dasp_index) for k in self.K) == 1), name="(20.1)")

        # Constraint (20.2)
        self.model.addConstr((gp.quicksum(gp.quicksum(self.x_k_ij[k, ds_dasp_index, j] for j in self.Vr_index
                                                      if j != ds_dasp_index) for k in self.K) == 1), name="(20.2)")

        # Aggiungi questa restrizione per bilanciare il flusso di ogni camion
        self.model.addConstrs(((gp.quicksum(self.x_k_ij[k, i, ds_dasp_index] for i in self.Vl_index if i != ds_dasp_index) == gp.quicksum(self.x_k_ij[k, ds_dasp_index, j] for j in self.Vr_index if j != ds_dasp_index)) for k in self.K), name = "flow_balance")


        # Constraint (21)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, i, os] for i in self.Vl_index if i != oe)
                                           for k in self.K) == 1 for (os, oe) in self.O_index), name="(21)")
        # Constraint (22)
        self.model.addConstrs((gp.quicksum(gp.quicksum(self.x_k_ij[k, oe, j] for j in self.Vr_index if j != os)
                                           for k in self.K) == 1 for (os, oe) in self.O_index), name="(22)")

        # Constraint (23)
        # self.model.addConstrs((gp.quicksum(self.x_k_ij[k, i, os] -
        #                                    self.x_k_ij[k, os, oe] for i in self.Vl_index) == 0
        #                        for k in self.K for (os, oe) in self.O_index), name="(23)")

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

    def set_start_variables(self):
        # print("Entro nel set_start_var, ecco i dasp_tours: ", self.dasp_tours)
        for k in range(len(self.dasp_tours)):
            start_node = self.V_start[k]
            start_node_tour = self.dasp_tours[k].index(start_node)

            # Usa un ciclo while per controllare l'indice i
            i = start_node_tour
            while i < len(self.dasp_tours[k]) - 1:
                node = self.dasp_tours[k][i]
                if node == self.V_end[k]:
                    break
                end_node = self.dasp_tours[k][i + 1]
                node_index = self.V.index(node)
                end_node_index = self.V.index(end_node)
                # print(f"{(k, i)} node_index: {node.index}, next node: {end_node.index}")

                self.x_k_ij[k + 1, node_index, end_node_index].Start = 1
                # print(f"set {k+1, node.index, end_node.index} = 1")

                if end_node_index in self.V_OS_index:
                    outlier_tuple = list(filter(lambda x: x[0] == end_node_index, self.O_index))[0]
                    outlier_end_index = outlier_tuple[1]
                    # print("outlier_end_index ", outlier_end_index)

                    outlier_end_dasp_index = self.dasp_tours[k].index(self.V[outlier_end_index])
                    # print("outlier_end_dasp_index ", outlier_end_dasp_index)

                    next_step = self.V.index(self.dasp_tours[k][outlier_end_dasp_index + 1])
                    # print("next_step ", next_step)

                    self.x_k_ij[k + 1, outlier_end_index, next_step].Start = 1
                    # print(f"set {k + 1, outlier_end_dasp_index, next_step + 1} = 1")

                    # Aggiorna l'indice i dopo aver processato l'outlier
                    i = outlier_end_dasp_index + 1
                else:
                    # Se non c'è un outlier, incrementa normalmente l'indice i
                    i += 1
