import itertools
import matplotlib.pyplot as plt


def generate_sub_tours_indexes(v):
    sub_tours_indexes = []
    for sub_tour_length in range(2, len(v)):
        for combo in itertools.combinations(v, sub_tour_length):
            sub_tours_indexes.append(list(combo))
    print(f" v = {v}, subtour_indexes: {sub_tours_indexes} ")
    return sub_tours_indexes


def getVisitedNodesIndex(tour):
    visited_nodes = []
    for var in tour:
        (starting_node, ending_node) = varToTupleIndex(var)
        if starting_node not in visited_nodes:
            visited_nodes.append(starting_node)
        if ending_node not in visited_nodes:
            visited_nodes.append(ending_node)
    return sorted(visited_nodes)


# From [x_k_ij] to [(i,j)]
def varToTupleIndex(var):
    parts = var.varName.split(",")
    starting_node = int(parts[1])
    ending_node = int(parts[2][:-1])
    return starting_node, ending_node


def tourToTuple(tour):
    ordered_tuple_tour = []
    tuple_tour = [varToTupleIndex(var) for var in tour]
    filtered_tuple = list(filter(lambda x: x[0] == 0, tuple_tour))[0]
    ordered_tuple_tour.append(filtered_tuple)
    tuple_tour.remove(filtered_tuple)  # remove visited tuple to improve efficiency
    for i in range(len(tuple_tour)):
        nextTuple = list(filter(lambda x: x[0] == filtered_tuple[1], tuple_tour))[0]
        ordered_tuple_tour.append(nextTuple)
        tuple_tour.remove(nextTuple)
        filtered_tuple = nextTuple
    return ordered_tuple_tour


def _getTrucksTour(model, decision_checker):
    k_var_lists = {}
    truck_k_tour = []
    for var in model._vars:
        if "x_k_ij" in var.varName:
            k = get_k_value(var.varName)  # Extract k value from variable name
            if k not in k_var_lists:
                k_var_lists[k] = []
            if decision_checker(var):
                k_var_lists[k].append(var)
    for k, var_list in k_var_lists.items():
        truck_k_tour.append(var_list)
    return truck_k_tour


def getTrucksTour_callback(model):
    return _getTrucksTour(model, lambda var: model.cbGetSolution(var) == 1)


def get_k_value(var_name):
    parts = var_name.split("[")
    return int(parts[1][0])
