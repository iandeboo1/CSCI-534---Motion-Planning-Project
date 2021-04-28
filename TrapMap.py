import shapely.geometry as sg
import math


# Trapezoid node class
class TrapNode(object):
    def __init__(self, ident):
        self.identifier = ident
        self.edges = [None, None, None, None]       # ordered LU, LL, RU, RL
        self.neighbors = [None, None, None, None]       # ordered LU, LL, RU, RL
        self.midpts = [None, None, None, None]       # ordered LU, LL, RU, RL
        self.visited = False


# Connectivity node class
class ConnNode(object):
    def __init__(self, ident):
        self.identifier = ident
        self.neighbors = []


# AVL tree class
class TrapMap(object):
    def __init__(self):
        self.traps = []
        self.conn_map = []

    def build_adjacency(self):
        if len(self.traps) != 0:
            self.get_midpoints()
            # seed = self.traps[0]
            self.find_adjacents()
            return self.conn_map

    def find_adjacents(self):
        for trap in self.traps:
            for i in range(len(trap.midpts)):
                if trap.midpts[i] is not None:
                    new_node = ConnNode(trap.midpts[i])
                    new_node.neighbors.append(trap)
                    new_node.neighbors.append(trap.neighbors[i])
                    self.conn_map.append(new_node)

    def get_midpoints(self):
        for trap in self.traps:
            for i in range(len(trap.neighbors)):
                if trap.neighbors[i] is not None:
                    edge = trap.edges[i]
                    y0 = edge[0].coords[0][1]
                    y1 = edge[0].coords[1][1]
                    y_dist = abs(y0 - y1)
                    if y0 < y1:
                        mid_y = y0 + (0.5 * y_dist)
                    else:
                        mid_y = y1 + (0.5 * y_dist)
                    factor = 10.0 ** 3
                    mid_y = math.trunc(mid_y * factor) / factor
                    trap.midpts[i] = sg.Point(edge[0].coords[0][0], mid_y)


