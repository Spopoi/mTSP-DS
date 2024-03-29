import numpy as np
import matplotlib.pyplot as plt
from MTSP_DS_Solver import MTSP_DS_Solver


def compute_average_solution(n, num_trials):
    execTimes = []
    for i in range(num_trials):
        solver = MTSP_DS_Solver(n, 0)
        solver.solve()
        execTime = solver.getExecTime()
        execTimes.append(execTime)
    average_exec_time = np.mean(execTimes, axis=0)
    return average_exec_time


def plot_results_for_n(values, num_trials=10):
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
    n_values = [4, 5, 6, 7, 8, 9, 10]
    plot_results_for_n(n_values)
