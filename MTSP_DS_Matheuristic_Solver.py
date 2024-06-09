import itertools
import time

from Local_DASP import Local_DASP
from MTSP_DS_MILP_Solver import MTSP_DS_MILP_Solver
from MTSP_DS_Solver import MTSP_DS_Solver


class MTSP_DS_Matheuristic_Solver(MTSP_DS_Solver):

    def __init__(self, n, m, Dn=2, Kn=1, C=None, alpha=1.2, eps=100, nodes=None, custom_locations=None):
        self.start_time = time.time()
        self.best_solution = None
        super().__init__(n, m, Dn, Kn, C, alpha, eps, nodes, custom_locations)
        self.d_station_combos = self.get_all_d_station_combos()
        self.execTime = 0

    def getTau(self):
        return self.best_solution

    def getExecTime(self):
        return self.execTime

    def get_all_d_station_combos(self):
        return list(itertools.combinations(self.Vs, self.C))

    def get_mtsp_nodes(self, d_station_combo):
        nodes_index = list(itertools.chain(self.Vn, d_station_combo))
        nodes = [self.v[i] for i in nodes_index if i == self.v[i].index]
        return nodes

    def get_mtsp_tours(self, drone_stations):
        nodes = self.get_mtsp_nodes(drone_stations)
        mtsp_solver = MTSP_DS_MILP_Solver(len(nodes), 0, 0, self.Kn, 0, self.alpha, self.eps, nodes=nodes)
        mtsp_solver.solve()

        mtsp_solver.plot_tours()
        tours = mtsp_solver.get_node_tours()
        for tour in tours:
            tour.append(self.v[-1])
        return tours

    def solve(self):
        solution_pool = []
        for d_station_combo in self.d_station_combos:
            solution = {"tours": [], "assigned_customers": [], "value": []}
            solution["tours"].extend(self.get_mtsp_tours(d_station_combo))
            for d_station in d_station_combo:
                local_dasp = Local_DASP(self, solution, d_station)
                solution = local_dasp.solve()
            solution_pool.append(solution)
        self.best_solution = min(solution_pool, key=lambda x: x["value"])
        end_time = time.time()
        self.execTime = end_time - self.start_time
