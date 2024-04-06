import itertools

import TourUtils
from MTSP_DS_MILP_Solver import MTSP_DS_MILP_Solver
from MTSP_DS_Solver import MTSP_DS_Solver


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
                solution = self.solve_local_dasp(solution, d_station)
            solution_pool.append(solution)
        print(solution_pool)

    def solve_local_dasp(self, solution, d_station):
        # print(d_station)
        starter_nodes = self.get_starter_nodes(solution, d_station)
        # for node in solution:
        # print("Nodo :", node)

    def get_starter_nodes(self, solution, d_station):
        starter_nodes = []
        # for each truck tour
        for i in range(len(solution)):
            # for each node
            for j in range(len(solution[i])):
                node = solution[i][j]
                # print(f"Station= {d_station} location = {self.v[d_station].location}")
                # print(f"Node {node.index} Location = {node.location}")
                # print("distance=", node.node_distance(self.v[d_station]))
                # print("eps=", self.eps / 2)
                # print("RESULT= ", node.node_distance(self.v[d_station]) <= self.eps / 2)
                if node != self.v[d_station] and node.node_distance(self.v[d_station]) <= self.eps / 2:
                    if j > 0:
                        if solution[i][j-1] not in starter_nodes:
                            starter_nodes.append(solution[i][j-1])
                    else:
                        if solution[i][j] not in starter_nodes:
                            starter_nodes.append(solution[i][j])
                    break
        # print(starter_nodes)
        return starter_nodes


if __name__ == "__main__":
    solver = MTSP_DS_Matheuristic_Solver(3, 4, 2, 2, 2)
    solver.solve()
