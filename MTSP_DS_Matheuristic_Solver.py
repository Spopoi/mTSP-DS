import itertools

from core import Location
from Local_DASP import Local_DASP
from MTSP_DS_MILP_Solver import MTSP_DS_MILP_Solver
from MTSP_DS_Solver import MTSP_DS_Solver


class MTSP_DS_Matheuristic_Solver(MTSP_DS_Solver):

    def __init__(self, n, m, Dn=2, Kn=1, C=None, alpha=1.2, eps=100, nodes=None, custom_locations=None):
        super().__init__(n, m, Dn, Kn, C, alpha, eps, nodes, custom_locations)
        self.d_station_combos = self.get_all_d_station_combos()
        # self.save_nodes_location_to_file("test_locations")

    def get_all_d_station_combos(self):
        return list(itertools.combinations(self.Vs, self.C))

    def get_mtsp_nodes(self, d_station_combo):
        nodes_index = list(itertools.chain(self.Vn, d_station_combo))
        nodes = [self.v[i] for i in nodes_index if i == self.v[i].index]
        return nodes

    def get_mtsp_tours(self, drone_stations):
        nodes = self.get_mtsp_nodes(drone_stations)
        mtsp_solver = MTSP_DS_MILP_Solver(len(nodes), 0, 0, self.Kn,
                                          0, self.alpha, self.eps, nodes=nodes)
        mtsp_solver.solve()
        print("mtsp_solution = ", mtsp_solver.getSolution())
        mtsp_solver.plot_tours()
        tours = mtsp_solver.NodesTour()
        for tour in tours:
            tour.append(self.v[-1])
        return tours

    def solve(self):
        solution_pool = []
        # print("d-combo", self.d_station_combos)
        for d_station_combo in self.d_station_combos:
            solution = {"tours": [], "assigned_customers": []}
            solution["tours"].extend(self.get_mtsp_tours(d_station_combo))
            # solution = self.get_mtsp_tours(d_station_combo)
            for d_station in d_station_combo:
                print(f"d_station: {d_station}, in d_station_combo: {d_station_combo}")
                # solution = self.solve_local_dasp(solution, d_station)
                local_dasp = Local_DASP(self, solution, d_station)
                solution = local_dasp.solve()
                # local_dasp.plotTours()
                print(f"Finito dasp con soluzione: {solution}")
            solution_pool.append(solution)
        print(solution_pool)


if __name__ == "__main__":
    locs = Location.create_custom_location_list("test_locations")
    print(locs)
    solver = MTSP_DS_Matheuristic_Solver(8, 1, 2, 2, 1, eps=150, custom_locations=locs)
    # solver = MTSP_DS_Matheuristic_Solver(8, 1, 2, 2, 1, eps=150)
    solver.solve()
