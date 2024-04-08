import itertools

from Local_DASP import Local_DASP
from MTSP_DS_MILP_Solver import MTSP_DS_MILP_Solver
from MTSP_DS_Solver import MTSP_DS_Solver
from Node import NodeType


class MTSP_DS_Matheuristic_Solver(MTSP_DS_Solver):

    def __init__(self, n, m, Dn=2, Kn=1, C=None, alpha=1.2, eps=100, nodes=None, custom_locations=None):
        super().__init__(n, m, Dn, Kn, C, alpha, eps, nodes, custom_locations)
        self.d_station_combos = self.get_all_d_station_combos()

    def get_all_d_station_combos(self):
        return list(itertools.combinations(self.Vs, self.C))

    def get_mtsp_nodes(self, d_station_combo):
        nodes_index = list(itertools.chain(self.Vn, d_station_combo))
        nodes = [self.v[i] for i in nodes_index if i == self.v[i].index]
        return nodes

    # def get_mtsp_locations(self, d_station_combo):
    #     nodes = list(itertools.chain(self.Vn, d_station_combo))
    #     return [self.v[i].location for i in nodes]

    def get_mtsp_tours(self, drone_stations):
        # locations = self.get_mtsp_locations(drone_stations)
        nodes = self.get_mtsp_nodes(drone_stations)
        mtsp_solver = MTSP_DS_MILP_Solver(len(nodes), 0, 0, self.Kn,
                                          0, self.alpha, self.eps, nodes=nodes)
        mtsp_solver.solve()
        mtsp_solver.plotTours()
        tours = mtsp_solver.NodesTour()
        for tour in tours:
            tour.append(self.v[-1])
        return tours

    def solve(self):
        solution_pool = []
        # print(self.d_station_combos)
        for d_station_combo in self.d_station_combos:
            solution = self.get_mtsp_tours(d_station_combo)
            for d_station in d_station_combo:
                # solution = self.solve_local_dasp(solution, d_station)
                local_dasp = Local_DASP(self, solution, d_station)
                solution = local_dasp.solve()
            solution_pool.append(solution)
        # print(solution_pool)

    # def solve_local_dasp(self, solution, d_station):
    #     # print(d_station)
    #     V_start = self.get_starter_nodes(solution, d_station)
    #     Vn = self.get_ds_custumers(solution, d_station)
    #     V_end = self.get_end_nodes(solution, d_station)
    #     Vos, Voe = self.get_outlier_nodes(solution, d_station, )
    #     # for node in solution:
    #     # print("Nodo :", node)


if __name__ == "__main__":
    solver = MTSP_DS_Matheuristic_Solver(9, 1, 2, 2, 1, eps=150)
    solver.solve()
