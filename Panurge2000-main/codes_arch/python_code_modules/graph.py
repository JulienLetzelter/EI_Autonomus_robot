import numpy as np
import random as rd


def set_graph(n, m=None):

    if m == None:
        m = n

    Graph = np.full((n*m, n*m), np.inf)
    Vit = np.zeros((n*m, n*m))
    v_max = 7

    for i in range(n*m): # setting the passes randomly
        if i % n != 0:
            t = rd.randint(1, 4)
            Vit[i, i-1] = v_max/t
            Vit[i-1, i] = v_max/t
            Graph[i, i-1] = t
            Graph[i-1, i] = t

        if i >= n:
            t = rd.randint(1, 4)
            Vit[i, i-n] = v_max/t
            Vit[i-n, i] = v_max/t
            Graph[i, i-n] = t
            Graph[i-n, i] = t

    return Graph

import numpy as np


def dijkstra(Graph, i, j):

    if i == j:
        return 0, []

    n = np.size(Graph, 1)
    frontier = [i]
    parent = {i: None}
    dist = {i: 0}

    while len(frontier) > 0:
        min_dist = np.inf
        x = frontier[0]
        for noeud in frontier:
            if dist[noeud] < min_dist:
                x = noeud
                min_dist = dist[noeud]
        frontier.remove(x)
        for y in range(n):
            if y not in parent:
                frontier.append(y)
            new_dist = dist[x] + Graph[x, y]
            if y not in dist or dist[y] > new_dist:
                dist[y] = new_dist
                parent[y] = x

    distance = dist[j]
    Chemin = []
    while j != i:
        Chemin.append(j)
        j = parent[j]

    return distance, Chemin # return distance and prectition( or the way in this case)