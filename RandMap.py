import shapely.geometry as sg
import TrapMap


# Cell node class
class CellNode(object):
    def __init__(self, height, coordinates):
        self.height = height
        self.label = None
        self.centroid = None
        self.coordinates = coordinates      # LU, LL, RU, RL
        self.children = [None, None, None, None]        # LU, LL, RU, RL   child cell order
        self.parent = None
        self.which_child = None


class RandMap(object):
    def __init__(self):
        self.root = None
        self.conn_map = []

    def build_adjacency(self, current_root):
        if current_root is not None:
            for child in current_root.children:
                if child is not None:
                    if child.label == "empty":
                        self.find_adjacents(child)
                    elif child.label == "mixed":
                        self.build_adjacency(child)
        if current_root is self.root:
            return self.conn_map

    def find_adjacents(self, cell):
        conn_node = TrapMap.ConnNode(sg.Point(cell.centroid))
        siblings = []
        for i in range(4):
            if i == 0:
                # Above neighbors
                if cell.which_child == 1:
                    siblings = siblings + self.find_above(cell.parent.children[0])
                elif cell.which_child == 3:
                    siblings = siblings + self.find_above(cell.parent.children[2])
                elif cell.which_child == 0 or cell.which_child == 2:
                    # Need to look elsewhere in quadtree
                    step_out = []
                    curr_cell = cell
                    while curr_cell.which_child == 0 or curr_cell.which_child == 2:
                        # Iterate up the tree until current cell is not one of the two top children
                        if curr_cell.parent is not None:
                            step_out.append(curr_cell.which_child)
                            curr_cell = curr_cell.parent
                        else:
                            step_out = None
                            break
                    if step_out is None:
                        # Original cell is on the top of the environment space, no above neighbors
                        continue
                    if curr_cell.parent is not None:
                        if curr_cell.which_child == 1:
                            curr_cell = curr_cell.parent.children[0]
                        else:
                            curr_cell = curr_cell.parent.children[2]
                        full_loop = True
                        while len(step_out) != 0:
                            # Iterate down the tree until neighbor is found
                            step_in = step_out.pop()
                            if curr_cell.children[0] is not None:
                                if step_in == 2:
                                    curr_cell = curr_cell.children[3]
                                else:
                                    curr_cell = curr_cell.children[1]
                            elif curr_cell.label == "empty":
                                # Neighbor found at higher level, doesn't have child cells
                                siblings = siblings + [curr_cell]
                                full_loop = False
                                break
                        if full_loop:
                            # Hit a neighboring cell at same level
                            siblings = siblings + self.find_above(curr_cell)
                    else:
                        # Original cell is on the top of the environment space, no above neighbors
                        continue
            elif i == 1:
                # Left neighbors
                if cell.which_child == 2:
                    siblings = siblings + self.find_left(cell.parent.children[0])
                elif cell.which_child == 3:
                    siblings = siblings + self.find_left(cell.parent.children[1])
                elif cell.which_child == 0 or cell.which_child == 1:
                    # Need to look elsewhere in quadtree
                    step_out = []
                    curr_cell = cell
                    while curr_cell.which_child == 0 or curr_cell.which_child == 1:
                        # Iterate up the tree until current cell is not one of the two left children
                        if curr_cell.parent is not None:
                            step_out.append(curr_cell.which_child)
                            curr_cell = curr_cell.parent
                        else:
                            step_out = None
                            break
                    if step_out is None:
                        # Original cell is on the left of the environment space, no left neighbors
                        continue
                    if curr_cell.parent is not None:
                        if curr_cell.which_child == 2:
                            curr_cell = curr_cell.parent.children[0]
                        else:
                            curr_cell = curr_cell.parent.children[1]
                        full_loop = True
                        while len(step_out) != 0:
                            # Iterate down the tree until neighbor is found
                            step_in = step_out.pop()
                            if curr_cell.children[0] is not None:
                                if step_in == 0:
                                    curr_cell = curr_cell.children[2]
                                else:
                                    curr_cell = curr_cell.children[3]
                            elif curr_cell.label == "empty":
                                # Neighbor found at higher level, doesn't have child cells
                                siblings = siblings + [curr_cell]
                                full_loop = False
                                break
                        if full_loop:
                            # Hit a neighboring cell at same level
                            siblings = siblings + self.find_left(curr_cell)
                    else:
                        # Original cell is on the left of the environment space, no left neighbors
                        continue
            elif i == 2:
                # Below neighbors
                if cell.which_child == 2:
                    siblings = siblings + self.find_below(cell.parent.children[3])
                elif cell.which_child == 0:
                    siblings = siblings + self.find_below(cell.parent.children[1])
                elif cell.which_child == 1 or cell.which_child == 3:
                    # Need to look elsewhere in quadtree
                    step_out = []
                    curr_cell = cell
                    while curr_cell.which_child == 1 or curr_cell.which_child == 3:
                        # Iterate up the tree until current cell is not one of the two bottom children
                        if curr_cell.parent is not None:
                            step_out.append(curr_cell.which_child)
                            curr_cell = curr_cell.parent
                        else:
                            step_out = None
                            break
                    if step_out is None:
                        # Original cell is on the bottom of the environment space, no below neighbors
                        continue
                    if curr_cell.parent is not None:
                        if curr_cell.which_child == 0:
                            curr_cell = curr_cell.parent.children[1]
                        else:
                            curr_cell = curr_cell.parent.children[3]
                        full_loop = True
                        while len(step_out) != 0:
                            # Iterate down the tree until neighbor is found
                            step_in = step_out.pop()
                            if curr_cell.children[0] is not None:
                                if step_in == 3:
                                    curr_cell = curr_cell.children[2]
                                else:
                                    curr_cell = curr_cell.children[0]
                            elif curr_cell.label == "empty":
                                # Neighbor found at higher level, doesn't have child cells
                                siblings = siblings + [curr_cell]
                                full_loop = False
                                break
                        if full_loop:
                            # Hit a neighboring cell at same level
                            siblings = siblings + self.find_below(curr_cell)
                    else:
                        # Original cell is on the bottom of the environment space, no below neighbors
                        continue
            else:
                # Right neighbors
                if cell.which_child == 0:
                    siblings = siblings + self.find_right(cell.parent.children[2])
                elif cell.which_child == 1:
                    siblings = siblings + self.find_right(cell.parent.children[3])
                elif cell.which_child == 2 or cell.which_child == 3:
                    # Need to look elsewhere in quadtree
                    step_out = []
                    curr_cell = cell
                    while curr_cell.which_child == 2 or curr_cell.which_child == 3:
                        # Iterate up the tree until current cell is not one of the two left children
                        if curr_cell.parent is not None:
                            step_out.append(curr_cell.which_child)
                            curr_cell = curr_cell.parent
                        else:
                            step_out = None
                            break
                    if step_out is None:
                        # Original cell is on the right of the environment space, no right neighbors
                        continue
                    if curr_cell.parent is not None:
                        if curr_cell.which_child == 0:
                            curr_cell = curr_cell.parent.children[2]
                        else:
                            curr_cell = curr_cell.parent.children[3]
                        full_loop = True
                        while len(step_out) != 0:
                            # Iterate down the tree until neighbor is found
                            step_in = step_out.pop()
                            if curr_cell.children[0] is not None:
                                if step_in == 2:
                                    curr_cell = curr_cell.children[0]
                                else:
                                    curr_cell = curr_cell.children[1]
                            elif curr_cell.label == "empty":
                                # Neighbor found at higher level, doesn't have child cells
                                siblings = siblings + [curr_cell]
                                full_loop = False
                                break
                        if full_loop:
                            # Hit a neighboring cell at same level
                            siblings = siblings + self.find_right(curr_cell)
                    else:
                        # Original cell is on the right of the environment space, no right neighbors
                        continue
        conn_node.neighbors = siblings
        self.conn_map.append(conn_node)

    def find_below(self, root_cell):
        below_cells = []
        if root_cell.children[0] is not None:
            ul_cells = self.find_below(root_cell.children[0])
            ur_cells = self.find_below(root_cell.children[2])
            if ul_cells is not None and ur_cells is not None:
                below_cells = ul_cells + ur_cells
            elif ul_cells is not None:
                below_cells = ul_cells
            elif ur_cells is not None:
                below_cells = ur_cells
            return below_cells
        else:
            if root_cell.label == "empty":
                return [root_cell]
            else:
                return []

    def find_above(self, root_cell):
        above_cells = []
        if root_cell.children[0] is not None:
            ll_cells = self.find_above(root_cell.children[1])
            lr_cells = self.find_above(root_cell.children[3])
            if ll_cells is not None and lr_cells is not None:
                above_cells = ll_cells + lr_cells
            elif ll_cells is not None:
                above_cells = ll_cells
            elif lr_cells is not None:
                above_cells = lr_cells
            return above_cells
        else:
            if root_cell.label == "empty":
                return [root_cell]
            else:
                return []

    def find_left(self, root_cell):
        left_cells = []
        if root_cell.children[0] is not None:
            ru_cells = self.find_left(root_cell.children[2])
            rl_cells = self.find_left(root_cell.children[3])
            if ru_cells is not None and rl_cells is not None:
                left_cells = ru_cells + rl_cells
            elif ru_cells is not None:
                left_cells = ru_cells
            elif rl_cells is not None:
                left_cells = rl_cells
            return left_cells
        else:
            if root_cell.label == "empty":
                return [root_cell]
            else:
                return []

    def find_right(self, root_cell):
        right_cells = []
        if root_cell.children[0] is not None:
            lu_cells = self.find_right(root_cell.children[0])
            ll_cells = self.find_right(root_cell.children[1])
            if lu_cells is not None and ll_cells is not None:
                right_cells = lu_cells + ll_cells
            elif lu_cells is not None:
                right_cells = lu_cells
            elif ll_cells is not None:
                right_cells = ll_cells
            return right_cells
        else:
            if root_cell.label == "empty":
                return [root_cell]
            else:
                return []
