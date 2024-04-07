import itertools

import TourUtils
from Customer import Customer
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
                solution = self.solve_local_dasp(solution, d_station)
            solution_pool.append(solution)
        # print(solution_pool)

    def solve_local_dasp(self, solution, d_station):
        # print(d_station)
        V_start = self.get_starter_nodes(solution, d_station)
        Vn = self.get_ds_custumers(solution, d_station)
        V_end = self.get_end_nodes(solution, d_station)
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
                        starter_nodes.append(solution[i][j - 1])
                    else:
                        starter_nodes.append(solution[i][j])
                    break
        # print(starter_nodes)
        return starter_nodes

    def get_ds_custumers(self, tours, d_station):
        Vn = []
        for tour in tours:
            for node in tour:
                if node.node_type == NodeType.CUSTOMER and node.node_distance(self.v[d_station]) <= self.eps / 2:
                    Vn.append(node)
        return Vn

    def get_end_nodes(self, solution, d_station):
        end_nodes = []
        # for each truck tour
        for i in range(len(solution)):
            # for each node
            for j in range(len(solution[i]) - 2, -1, -1):
                # print(f"({i},{j}):\n Dims = {len(solution)}, {len(solution[i])}")
                node = solution[i][j]
                # if last step is ds -> set depot as end node
                if j == len(solution[i]) - 2 and node == self.v[d_station]:
                    end_nodes.append(solution[i][len(solution[i]) - 1])
                    break
                if node != self.v[d_station] and node.node_distance(self.v[d_station]) <= self.eps / 2:
                    end_nodes.append(solution[i][j + 1])
                    break
        # print("end nodes: ", end_nodes)
        return end_nodes


if __name__ == "__main__":
    solver = MTSP_DS_Matheuristic_Solver(5, 1, 2, 2, 1, eps=200)
    solver.solve()
