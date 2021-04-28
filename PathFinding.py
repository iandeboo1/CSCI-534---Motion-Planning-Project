import shapely.geometry as sg
import heapq


def build_graph(adj_list, mode):
    graph = {}
    if mode == 0:
        # Trapezoid cell mode
        for node in adj_list:
            if node.identifier.bounds not in graph.keys():
                value = []
                for neighbor in node.neighbors:
                    j = None
                    for i in range(len(neighbor.midpts)):
                        if neighbor.midpts[i] is not None:
                            if neighbor.midpts[i].bounds == node.identifier.bounds:
                                j = i
                    if j == 0 or j == 1:
                        if neighbor.midpts[2] is not None:
                            value.append(neighbor.midpts[2])
                        if neighbor.midpts[3] is not None:
                            value.append(neighbor.midpts[3])
                    else:
                        if neighbor.midpts[0] is not None:
                            value.append(neighbor.midpts[0])
                        if neighbor.midpts[1] is not None:
                            value.append(neighbor.midpts[1])
                graph[node.identifier.bounds] = value
    else:
        # Random cell mode
        for node in adj_list:
            value = []
            if node.neighbors is not None:
                for cell in node.neighbors:
                    value.append(sg.Point(cell.centroid))
                graph[node.identifier.bounds] = value
    return graph


def find_path(s_node, t_node, graph):
    queue = []
    dist = {}
    prev = {}
    visited = []
    for node in graph.keys():
        dist[node] = 0
        prev[node] = None
    heapq.heappush(queue, (dist[s_node.bounds], s_node.bounds))

    while len(queue) != 0:
        u = heapq.heappop(queue)
        visited.append(sg.Point(u[1][0], u[1][1]))
        if u[1] != t_node.bounds:
            for neighbor in graph[u[1]]:
                if neighbor not in visited:
                    u_pt = sg.Point(u[1][0], u[1][1])
                    alt = dist[u[1]] + u_pt.distance(neighbor)
                    if alt < dist[neighbor.bounds] or dist[neighbor.bounds] == 0:
                        dist[neighbor.bounds] = alt
                        prev[neighbor.bounds] = u[1]
                        if neighbor.bounds not in queue:
                            heapq.heappush(queue, (dist[neighbor.bounds], neighbor.bounds))
        else:
            # t_node found
            path = []
            total_dist = 0
            u = u[1]
            path.append(u)
            if prev[u] is not None or u == s_node.bounds:
                while u is not None and u != s_node.bounds:
                    path.append(prev[u])
                    pt_prev = sg.Point(prev[u][0], prev[u][1])
                    pt_curr = sg.Point(u[0], u[1])
                    edge_dist = pt_prev.distance(pt_curr)
                    total_dist += edge_dist
                    u = prev[u]
                rev_path = path[::-1]
                pt_path = []
                for node in rev_path:
                    pt_path.append(sg.Point(node[0], node[1]))
                return pt_path, total_dist
    return None, None


def locate_cell_pts(actual_pt, mode, cell_map):
    if mode == 0:
        # Trapezoid cell mode
        min_dist = 0
        closest_pt = None
        for trap in cell_map.traps:
            for pt in trap.midpts:
                if pt is not None:
                    difference = actual_pt.distance(pt)
                    if difference < min_dist or min_dist == 0:
                        min_dist = difference
                        closest_pt = pt
        return closest_pt
    else:
        # Random cell mode
        return find_approx_cell(cell_map.root, actual_pt)


def find_approx_cell(curr_root, actual_pt):
    curr_poly = sg.Polygon([curr_root.coordinates[0], curr_root.coordinates[1], curr_root.coordinates[3],
                           curr_root.coordinates[2], curr_root.coordinates[0]])
    if actual_pt.intersects(curr_poly):
        for child in curr_root.children:
            if child is not None:
                temp = find_approx_cell(child, actual_pt)
                if temp is not None:
                    return temp
        return sg.Point(curr_root.centroid[0], curr_root.centroid[1])
    else:
        return None
