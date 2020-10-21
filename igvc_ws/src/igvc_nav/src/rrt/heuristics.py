import numpy as np

def travel_cost(a, b):
    return np.linalg.norm(np.array(b) - np.array(a))

def path_cost(edges, a, b):
    cost = 0
    while not b == a:
        p = edges[b]
        cost += travel_cost(a, b)
        b = p
    return cost
