import itertools

import numpy as np
from matplotlib import pyplot as plt

from MTSP_DS_MILP_Solver import MTSP_DS_MILP_Solver
from MTSP_DS_Solver import MTSP_DS_Solver


class MTSP_DS_Matheuristic_Solver(MTSP_DS_Solver):
    maxLocationBound = 150

    def __init__(self, n, m, Dn=2, Kn=1, C=None, alpha=1.2, eps=100, custom_locations=None):
        super().__init__(n, m, Dn, Kn, C, alpha, eps, custom_locations)
        self.d_station_combos = self.get_all_d_station_combos()

    def get_all_d_station_combos(self):
        return list(itertools.combinations(self.Vs, self.C))

    # def solve(self):
    # for d_station_combo in self.d_station_combos:

    def get_mtsp_tours(self, drone_stations):
        mtsp_solver = MTSP_DS_MILP_Solver(self.n, self.m, self.Dn, self.Kn,
                                          self.C, self.alpha, self.eps)
        mtsp_solver.solve()
        solution = mtsp_solver.getTrucksTour()
        print(solution)
        mtsp_solver.plotTours()
        # solution_tours


if __name__ == "__main__":
    solver = MTSP_DS_Matheuristic_Solver(5, 2, 2, 2)
    solver.get_mtsp_tours(None)
    print(solver.get_all_d_station_combos())
