import os
import numpy as np

from core.Location import Location


def read_vrp_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    section = None
    capacity = 0
    demand = []
    coordinates = []
    depot = None

    for line in lines:
        line = line.strip()

        if line.startswith('CAPACITY'):
            capacity = int(line.split()[-1])
        elif line.startswith('NODE_COORD_SECTION'):
            section = 'NODE_COORD_SECTION'
        elif line.startswith('DEMAND_SECTION'):
            section = 'DEMAND_SECTION'
        elif line.startswith('DEPOT_SECTION'):
            section = 'DEPOT_SECTION'
        elif line.startswith('EOF'):
            break
        elif section == 'NODE_COORD_SECTION':
            parts = line.split()
            coordinates.append((int(parts[1]), int(parts[2])))
        elif section == 'DEMAND_SECTION':
            parts = line.split()
            demand.append(int(parts[1]))
        elif section == 'DEPOT_SECTION':
            depot = int(line)

    coordinates = np.array(coordinates)
    demand = np.array(demand)

    return capacity, coordinates, demand, depot


def load_augerat_instances(base_path):
    instances = {'A': [], 'B': [], 'P': []}

    for category in instances.keys():
        category_path = os.path.join(base_path, category)
        for file_name in os.listdir(category_path):
            if file_name.endswith('.vrp'):
                file_path = os.path.join(category_path, file_name)
                capacity, coordinates, demand, depot = read_vrp_file(file_path)
                instances[category].append({
                    'file_name': file_name,
                    'capacity': capacity,
                    'coordinates': coordinates,
                    'demand': demand,
                    'depot': depot
                })

    return instances


def get_locations(dataset_name='A', instance_number=0):
    dataset = get_dataset()
    instance = dataset[dataset_name][instance_number]
    coordinates = instance['coordinates']
    # print("COOORDINATE: ", coordinates)
    locations = []
    for coordinate in coordinates:
        locations.append(Location(coordinate[0], coordinate[1]))
    return locations


def get_dataset():
    base_path = os.path.join('..', 'data')
    return load_augerat_instances(base_path)
