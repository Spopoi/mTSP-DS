import itertools
import matplotlib.pyplot as plt

from Customer import Customer
from DroneStation import DroneStation
from Node import NodeType


def generate_sub_tours_indexes(v):
    sub_tours_indexes = []
    for sub_tour_length in range(2, len(v)):
        for combo in itertools.combinations(v, sub_tour_length):
            sub_tours_indexes.append(list(combo))
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


def varToCustomerDroneIndex(var):
    parts = var.varName.split(",")
    drone_index = int(parts[1][0])
    return drone_index


def tourToTuple(tour):
    ordered_tuple_tour = []
    tuple_tour = [varToTupleIndex(var) for var in tour]
    # print("ECCOLO IL TUPLETOUR: ", tuple_tour)
    filtered_tuple = list(filter(lambda x: x[0] == 0, tuple_tour))[0]
    ordered_tuple_tour.append(filtered_tuple)
    tuple_tour.remove(filtered_tuple)
    for i in range(len(tuple_tour)):
        nextTuple = list(filter(lambda x: x[0] == filtered_tuple[1], tuple_tour))[0]
        ordered_tuple_tour.append(nextTuple)
        tuple_tour.remove(nextTuple)
        filtered_tuple = nextTuple
    return ordered_tuple_tour


def get_customer_drone_edges(model):
    drone_to_customers_edges = []
    for var in model._vars:
        if "y_d_sj" in var.varName:
            if var.x == 1:
                drone_to_customers_edges.append(varToTupleIndex(var))
    return drone_to_customers_edges


def _getTrucksTour(vars, decision_checker):
    k_var_lists = {}
    truck_k_tour = []
    for var in vars:
        if "x_k_ij" in var.varName:
            k = get_k_value(var.varName)
            if k not in k_var_lists:
                k_var_lists[k] = []
            if decision_checker(var):
                k_var_lists[k].append(var)
    for k, var_list in k_var_lists.items():
        truck_k_tour.append(var_list)
    return truck_k_tour


def getTrucksTour_callback(model):
    return _getTrucksTour(model._vars, lambda var: model.cbGetSolution(var) == 1)


def getTrucksTour(model):
    return _getTrucksTour(model._vars, lambda var: var.x == 1)


def get_k_value(var_name):
    parts = var_name.split("[")
    return int(parts[1][0])


def getTupleTour(model):
    tours = getTrucksTour(model)
    tuple_tours = []
    for tour in tours:
        tuples_tour = tourToTuple(tour)
        tuple_tours.append(tuples_tour)
    return tuple_tours


def plotTours(model, v, eps):
    tuple_tours = getTupleTour(model)
    drone_deliveries = get_customer_drone_edges(model)
    drone_deliveries_from_s = None
    for tour in tuple_tours:
        for i, _ in tour:
            if v[i].node_type == NodeType.DRONE_STATION:
                drone_deliveries_from_s = list(filter(lambda x: x[0] == i, drone_deliveries))
        plotNodes(v, eps, tour, drone_deliveries_from_s)


def plotNodes(v, eps, tour_tuples=None, drone_deliveries=None):
    x_values = [node.location.x for node in v]
    y_values = [node.location.y for node in v]

    node_types = [
        'customer' if isinstance(node, Customer) else 'drone_station' if isinstance(node, DroneStation) else 'depot'
        for node in v]

    plt.scatter(x_values, y_values, color='black', label="Nodes")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Nodes plot")

    for node, node_type in zip(v[:-1], node_types[:-1]):
        plt.annotate(str(node.index), (node.location.x, node.location.y), textcoords="offset points",
                     xytext=(0, 10),
                     ha='center')

    for node, node_type in zip(v, node_types):
        if node_type == 'customer':
            plt.plot(node.location.x, node.location.y, 'bo', markersize=8)
        elif node_type == 'drone_station':
            plt.plot(node.location.x, node.location.y, 'r*', markersize=10)
        else:  # depot
            plt.plot(node.location.x, node.location.y, 'kD', markersize=7)

    blue_patch = plt.Line2D([0], [0], marker='o', color='w', label='Customer', markerfacecolor='blue',
                            markersize=10)
    red_patch = plt.Line2D([0], [0], marker='s', color='w', label='Drone Station', markerfacecolor='red',
                           markersize=10)
    black_patch = plt.Line2D([0], [0], marker='o', color='w', label='Depot', markerfacecolor='black', markersize=10)
    green_patch = plt.Line2D([0], [0], marker='_', color='g', label='Drone delivery', markerfacecolor='green',
                             markersize=10)
    tour_patch = plt.Line2D([0], [0], marker='_', color='black', label='Truck tour', markerfacecolor='black',
                            markersize=10)

    plt.legend(handles=[black_patch, red_patch, blue_patch, green_patch, tour_patch])

    for node, node_type in zip(v, node_types):
        if node_type == 'drone_station':
            circle = plt.Circle((node.location.x, node.location.y), eps / 2, color='red', fill=False,
                                linestyle='dashed')
            plt.gca().add_patch(circle)

    if tour_tuples:
        for edge in tour_tuples:
            start_node = v[edge[0]]
            end_node = v[edge[1]]
            plt.arrow(start_node.location.x, start_node.location.y,
                      end_node.location.x - start_node.location.x,
                      end_node.location.y - start_node.location.y,
                      head_width=0.2, head_length=0.2, fc='black', ec='black')
    if drone_deliveries:
        for edge in drone_deliveries:
            start_node = v[edge[0]]
            end_node = v[edge[1]]
            plt.arrow(start_node.location.x, start_node.location.y,
                      end_node.location.x - start_node.location.x,
                      end_node.location.y - start_node.location.y,
                      head_width=0.2, head_length=0.2, fc='green', ec='green')
    plt.show()
