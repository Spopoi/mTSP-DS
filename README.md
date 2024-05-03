# mTSP-DS Solver

Welcome to the GitHub repository for the mTSP-DS (Multiple Traveler Salesman Problem in Presence of Drone Stations) project. This project is based on the paper "The Multiple Traveling Salesman Problem in Presence of Drone- and Robot-Supported Packet Stations" by Konstantin Kloster, Mahdi Moeini, Daniele Vigo, and Oliver Wendt.

This repository contains two main implementations:

- **MILP Version**: A version of the problem implemented using MILP (Mixed Integer Linear Programming) to solve smaller problem instances.
- **Matheuristic Version**: A matheuristic (a combined approach of exact and heuristic methods) for solving larger problem instances.

## Requirements

To run this project, you will need the following:

- **Python** (version 3.8 or higher recommended)
- **Gurobi Optimizer**: Obtain a license and installation from [here](https://www.gurobi.com/downloads/).
- **gurobipy**: The Python interface for Gurobi Optimizer.

## Installation

1. **Clone this repository**:

    ```shell
    git clone https://github.com/Spopoi/mTSP-DS.git
    ```

2. **Install dependencies**:

    You can install `gurobipy` using `pip`:

    ```shell
    pip install gurobipy
    ```

3. **Configure Gurobi**:

    Make sure you have properly configured Gurobi Optimizer and `gurobipy` on your system.

## Running the Code

You can run the provided scripts to solve MTSP-DS problems:

- **MILP Version**: Run the script `MTSP_DS_MILP_Solver.py` from the `src/` directory:

    ```shell
    python src/MTSP_DS_MILP_Solver.py
    ```

- **Matheuristic Version**: Run the script `MTSP_DS_Matheuristic_Solver.py` from the `src/` directory:

    ```shell
    python src/MTSP_DS_Matheuristic_Solver.py
    ```

Adjust the parameters in the script files as needed to match your problem instances.

## Contributing

If you would like to contribute to this project, feel free to submit pull requests or open issues to report bugs or suggest improvements.
