import numpy as np
import matplotlib.pyplot as plt

from Location import Location
from MTSP_DS_Solver import MTSP_DS_Solver


def compute_average_solution(n, num_trials):
    execTimes = []
    for i in range(num_trials):
        loc = [Location(100, 130), Location(100, 90), Location(120, 100), Location(100, 100), Location(49,49)]
        solver = MTSP_DS_Solver(3, 2, 2, 2, alpha=4, custom_locations=loc)

        solver.solve()
        execTime = solver.getExecTime()
        solver.printExecutionLog()
        execTimes.append(execTime)
        solver.plotNodes()
        solver.plotTours()
    average_exec_time = np.mean(execTimes, axis=0)
    return average_exec_time


def plot_results_for_n(values, num_trials=1):
    average_solutions = []
    for n in values:
        average_solution = compute_average_solution(n, num_trials)
        average_solutions.append(average_solution)

    plt.plot(values, average_solutions, marker='o')
    plt.xlabel('Number of Nodes (n)')
    plt.ylabel('Average Execution time')
    plt.title('Average MTSP Execution Time for Different Number of Nodes')
    plt.grid(True)
    # plt.savefig('mtsp_ds_sensitivityAnalysis.png')
    plt.show()


if __name__ == "__main__":
    n_values = [5, 10]
    plot_results_for_n(n_values)

