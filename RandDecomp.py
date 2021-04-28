import shapely.geometry as sg
import RandMap
import math

rm = RandMap.RandMap()


def strset_to_poly(obstacle_set):
    poly_set = []
    for obstacle in obstacle_set:
        temp = []
        for seg in obstacle:
            temp.append(seg.coords[0])
        poly = sg.Polygon(temp)
        poly_set.append(poly)
    return poly_set


def get_bounding_space_cell(poly):
    bounds = poly.bounds
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    center = ((width / 2) + bounds[0], (height / 2) + bounds[1])
    if width > height:
        points = [(bounds[0], center[1] + (width / 2)), (bounds[0], center[1] - (width / 2)),
                  (bounds[2], center[1] + (width / 2)), (bounds[2], center[1] - (width / 2))]       # LU, LL, RU, RL
        return width, points
    else:
        points = [(center[0] - (height / 2), bounds[3]), (center[0] - (height / 2), bounds[1]),
                  (center[0] + (height / 2), bounds[3]), (center[0] + (height / 2), bounds[1])]     # LU, LL, RU, RL
        return height, points


def decompose(cell, obstacles, total_height, resolution):
    cell_poly = sg.Polygon([cell.coordinates[0], cell.coordinates[1], cell.coordinates[3], cell.coordinates[2]])
    if cell_poly.intersects(obstacles[0]):
        # Cell is at least partially within the environment boundary
        dim = total_height / (2 ** cell.height)
        centroid = (cell.coordinates[0][0] + (dim / 2), cell.coordinates[0][1] - (dim / 2))
        factor = 10.0 ** 3
        centroid = (math.trunc(centroid[0] * factor) / factor, math.trunc(centroid[1] * factor) / factor)
        for obstacle in obstacles[1:]:
            if cell_poly.within(obstacle):
                cell.label = "full"
                return
            elif cell_poly.intersects(obstacle) or (cell_poly.intersects(obstacles[0]) and not cell_poly.within(obstacles[0])):
                cell.label = "mixed"
                if (total_height / (2 ** (cell.height + 1))) > resolution:
                    # Children will not be below min resolution limit
                    for i in range(4):      # LU, LL, RU, RL   child cell order
                        new_coords = []     # LU, LL, RU, RL
                        if i == 0:
                            new_coords = [cell.coordinates[0], (cell.coordinates[0][0], cell.coordinates[0][1] - (dim / 2)),
                                          (cell.coordinates[0][0] + (dim / 2), cell.coordinates[0][1]), centroid]
                        elif i == 1:
                            new_coords = [(cell.coordinates[0][0], cell.coordinates[0][1] - (dim / 2)), cell.coordinates[1],
                                          centroid, (cell.coordinates[1][0] + (dim / 2), cell.coordinates[1][1])]
                        elif i == 2:
                            new_coords = [(cell.coordinates[0][0] + (dim / 2), cell.coordinates[0][1]), centroid,
                                          cell.coordinates[2], (cell.coordinates[2][0], cell.coordinates[2][1] - (dim / 2))]
                        else:
                            new_coords = [centroid, (cell.coordinates[1][0] + (dim / 2), cell.coordinates[1][1]),
                                          (cell.coordinates[2][0], cell.coordinates[2][1] - (dim / 2)), cell.coordinates[3]]
                        new_cell = RandMap.CellNode(cell.height + 1, new_coords)
                        new_cell.parent = cell
                        new_cell.which_child = i
                        cell.children[i] = new_cell
                    for child in cell.children:
                        decompose(child, obstacles, total_height, resolution)
                if cell.height == 0:
                    rm.root = cell
                    return rm
                else:
                    return
        # For loop completed without returning
        cell.label = "empty"
        cell.centroid = centroid
        return
    else:
        # Cell is outside boundary
        cell.label = "outside"
        return




