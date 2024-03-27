import itertools
from gurobipy import GRB
import gurobipy as gp


def generate_sub_tours_indexes(v):
    sub_tours_indexes = []
    for sub_tour_length in range(2, len(v) - 1):
        for combo in itertools.combinations(v[1:-1], sub_tour_length):
            sub_tours_indexes.append(list(combo))
    return sub_tours_indexes


def getVisitedNodesIndex(tour):
    visited_nodes = []
    for var in tour:
        parts = var.varName.split(",")
        starting_node = int(parts[1])
        ending_node = int(parts[2][:-1])
        if starting_node not in visited_nodes:
            visited_nodes.append(starting_node)
        if ending_node not in visited_nodes:
            visited_nodes.append(ending_node)
    return sorted(visited_nodes)


def get_k_value(var_name):
    parts = var_name.split("[")
    return int(parts[1][0])

#
# def getTrucksTour(model):
#     k_var_lists = {}
#     truck_k_tour = []
#     for var in model._vars:
#         if "x_k_ij" in var.varName:
#             k = get_k_value(var.varName)  # Extract k value from variable name
#             if k not in k_var_lists:
#                 k_var_lists[k] = []
#             if model.cbGetSolution(var) == 1:
#                 k_var_lists[k].append(var)
#     for k, var_list in k_var_lists.items():
#         truck_k_tour.append(var_list)
#         # print(f"Variables for k = {k}: {var_list}")
#     return truck_k_tour
#
#
# def subtourelim(model, where):
#     if where == GRB.Callback.MIPSOL:
#         tours = getTrucksTour(model)
#         truck_index = 1
#         x_k_ij = model._edges
#         for truck_tour in tours:
#             node_indexes = getVisitedNodesIndex(truck_tour)
#             sub_tours_indexes = generate_sub_tours_indexes(node_indexes)
#             # Constraint (13)
#             for S in sub_tours_indexes:
#                 # print(S)
#                 print(gp.quicksum(gp.quicksum(x_k_ij[truck_index, i, j] for j in S if i != j) for i in S) <= len(S) - 1)
#                 model.cbLazy(
#                     gp.quicksum(gp.quicksum(x_k_ij[truck_index, i, j] for j in S if i != j) for i in S)
#                     <= len(S) - 1)
#             truck_index += 1
#             model.update()
