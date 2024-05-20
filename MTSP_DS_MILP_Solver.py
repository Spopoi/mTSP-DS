import itertools

import gurobipy as gp
import numpy as np
from gurobipy import GRB

from MTSP_DS_Solver import MTSP_DS_Solver
from TourUtils import getVisitedNodesIndex, generate_sub_tours_indexes, getTrucksTour_callback, plotTours


class MTSP_DS_MILP_Solver(MTSP_DS_Solver):

    def __init__(self, n, m, Dn=2, Kn=1, C=None, alpha=1.2, eps=100, nodes=None, custom_locations=None):
        super().__init__(n, m, Dn, Kn, C, alpha, eps, nodes, custom_locations)

        self.V = np.arange(len(self.v))
        print("MILP v: ", self.v)
        print("MILP V: ", self.V)
        print("MILP Vn: ", self.Vn)
        self.Vl = self.V[:-1]
        self.Vr = self.V[1:]
        self.H = list(itertools.chain(self.Vn, self.Vs))  # Vn u Vs

        # init decision variables
        self.y_d_sj = None
        self.a_ki = None
        self.z_s = None
        self.x_k_ij = None
        self.tau = None
        self.model = None
        self.initModel()

    # TODO: move
    def plot_tours(self):
        plotTours(self.model, self.v, self.eps)

    def initModel(self):
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

        customer_ds_indexes = list(itertools.chain(self.Vn, self.Vs))
        print("indiciiii: ", customer_ds_indexes)

        # Constraint (5.1)
        self.model.addConstrs((gp.quicksum(self.x_k_ij[k, 0, j] for j in customer_ds_indexes) == 1 for k in self.K), name="(5.1)")

        # Constraint (5.2)
        self.model.addConstrs(
            (gp.quicksum(self.x_k_ij[k, i, 1 + self.n + self.m] for i in customer_ds_indexes) == 1 for k in self.K), name="(5.2)")

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

    def subtourelim(self, model, where):
        if where == GRB.Callback.MIPSOL:
            tours = getTrucksTour_callback(model)
            truck_index = 1
            x_k_ij = model._edges
            for truck_tour in tours:
                node_indexes = getVisitedNodesIndex(truck_tour)
                sub_tours_indexes = generate_sub_tours_indexes(node_indexes[1:-1])
                # Constraint (13)
                for S in sub_tours_indexes:
                    model.cbLazy(
                        gp.quicksum(gp.quicksum(x_k_ij[truck_index, i, j] for j in S if i != j) for i in S)
                        <= len(S) - 1)
                    model.update()
                truck_index += 1

    def solve(self):
        self.model.optimize(self.subtourelim)
