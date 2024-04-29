import math

import numpy as np


def create_custom_location_list(filepath: str):
    custom_location_list = []

    # Apri il file in modalità lettura
    with open(filepath, 'r') as file:
        # Leggi il file riga per riga
        for line in file:
            # Rimuovi eventuali spazi bianchi e parentesi dalla riga
            line = line.replace("(", "").replace(")", "").strip()
            # Suddividi la riga in coordinate usando lo spazio come delimitatore
            coordinates = line.split(" ")
            # Per ciascuna coppia di coordinate nella riga
            for coordinate in coordinates:
                # Suddividi la coppia in latitudine e longitudine
                latitude, longitude = map(float, coordinate.split(","))
                # Crea un oggetto Location con i valori di latitudine e longitudine
                location = Location(latitude, longitude)
                # Aggiungi l'oggetto Location alla lista
                custom_location_list.append(location)

    # Restituisci la lista di oggetti Location
    return custom_location_list


def rand_location(maxLocationBound=150):
    rand_x = np.random.randint(1, maxLocationBound)
    rand_y = np.random.randint(1, maxLocationBound)
    return Location(rand_x, rand_y)


class Location:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"({self.x},{self.y})"

    def euclidean_distance(self, other_location):
        delta_x = self.x - other_location.x
        delt_y = self.y - other_location.y
        return math.sqrt(delta_x * delta_x + delt_y * delt_y)
