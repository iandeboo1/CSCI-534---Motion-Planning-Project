import shapely.geometry as sg
from queue import PriorityQueue
import TrapMap
import math

seg_dict = {}
q = PriorityQueue()


def extract_endpoints(obstacles):
    seg_dict.clear()
    # Iterate through obstacles and environment boundary
    for obstacle in obstacles:
        # Iterate through segments of obstacle
        for seg in obstacle:
            if seg.coords[0] in seg_dict:
                # Another segment is already associated with this endpoint
                curr_seg = seg_dict[seg.coords[0]]
                seg_dict[seg.coords[0]] = [seg, curr_seg]
            else:
                # No other segment associated with this endpoint
                seg_dict[seg.coords[0]] = seg
            if seg.coords[1] in seg_dict:
                # Another segment is already associated with this endpoint
                curr_seg = seg_dict[seg.coords[1]]
                seg_dict[seg.coords[1]] = [seg, curr_seg]
            else:
                # No other segment associated with this endpoint
                seg_dict[seg.coords[1]] = seg
    return seg_dict


def merge_sort(pts):
    if len(pts) > 1:
        midpt = len(pts)//2
        left = pts[:midpt]
        right = pts[midpt:]
        merge_sort(left)
        merge_sort(right)

        i, j, k = 0, 0, 0

        while i < len(left) and j < len(right):
            if left[i][0] < right[j][0]:
                pts[k] = left[i]
                i += 1
            else:
                pts[k] = right[j]
                j += 1
            k += 1

        while i < len(left):
            pts[k] = left[i]
            i += 1
            k += 1

        while j < len(right):
            pts[k] = right[j]
            j += 1
            k += 1
    return pts


def find_intersections(ev_vert, s_line):
    if len(s_line) == 0:
        return None
    else:
        above_val = None
        above_line = None
        below_val = None
        below_line = None
        for line in s_line:
            c1 = line.coords[0]
            c2 = line.coords[1]
            if ev_vert != c1 and ev_vert != c2:
                a = (c2[1] - c1[1]) / (c2[0] - c1[0])
                b = (c2[1] - (a * c2[0]))
                y = ((a*ev_vert[0]) + b)
                if (ev_vert[1] - y) > 0:
                    # line is below event vertex
                    if below_val is None or y > below_val:
                        below_val = y
                        below_line = line
                elif (ev_vert[1] - y) < 0:
                    # line is above event vertex
                    if above_val is None or y < above_val:
                        above_val = y
                        above_line = line
        return [(ev_vert[0], above_val), (ev_vert[0], below_val), above_line, below_line]


def slope(x1, y1, x2, y2):
    return (y2-y1)/(x2-x1)


def is_upper_line(line1, line2):
    # Find slopes
    slope1 = slope(line1.coords[0][0], line1.coords[0][1], line1.coords[1][0], line1.coords[1][1])
    slope2 = slope(line2.coords[0][0], line2.coords[0][1], line2.coords[1][0], line2.coords[1][1])
    if slope1 > slope2:
        return True
    else:
        return False


def is_convex(obstacles, line1, line2):
    for obstacle in obstacles:
        temp = []
        for seg in obstacle:
            temp.append(seg.coords[0])
        poly = sg.Polygon(temp)
        if line1.within(poly) or line2.within(poly):
            return True
    return False


def decompose(obstacle_points, obs):
    concave = False
    max_y = 0
    min_y = 0
    for pt in obstacle_points:
        if pt[1] > max_y:
            max_y = pt[1]
        if pt[1] < min_y:
            min_y = pt[1]
        q.put(pt)

    s_line = []

    tm = TrapMap.TrapMap()
    open_traps = {}     # line : trapezoid   format
    trap_id = 1

    # Sweep across environment space
    while not q.empty():
        nxt = q.get()
        upper_divider = None
        lower_divider = None

        # Get opposite coordinates for both lines connected to event vertex
        for coords in seg_dict[nxt][0].coords:
            if coords != nxt:
                other_coords0 = coords
        for coords in seg_dict[nxt][1].coords:
            if coords != nxt:
                other_coords1 = coords

        # Find closest segments above and below in sweep line
        # [(above intersection point), (below intersection point), above_line, below_line] format
        adj_lines = find_intersections(nxt, s_line)

        # Handle 4 event types
        if other_coords0[0] < nxt[0] and other_coords1[0] < nxt[0]:
            # Both adjacent lines are to the left of the sweep line
            s_line.remove(seg_dict[nxt][0])
            s_line.remove(seg_dict[nxt][1])
            if adj_lines is not None and len(s_line) != 0:
                if adj_lines[0][0] is not None and adj_lines[0][1] is not None and adj_lines[1][0] is not None and adj_lines[1][1] is not None:
                    upper_divider = sg.MultiLineString([(nxt, adj_lines[0])])
                    lower_divider = sg.MultiLineString([(nxt, adj_lines[1])])
                else:
                    # Close trapezoid to the left
                    open_traps.pop(seg_dict[nxt][0].bounds)
                    open_traps.pop(seg_dict[nxt][1].bounds)
                    continue
                if not is_convex(obs, upper_divider, lower_divider):
                    # Close two trapezoids to the left
                    open_traps.pop(seg_dict[nxt][0].bounds)
                    open_traps.pop(seg_dict[nxt][1].bounds)
                    prev_1 = open_traps.pop(adj_lines[2].bounds)
                    prev_2 = open_traps.pop(adj_lines[3].bounds)
                    prev_1.edges[2] = upper_divider
                    prev_2.edges[2] = lower_divider
                    # Create new trapezoid
                    new_trap = TrapMap.TrapNode(trap_id)
                    new_trap.edges[0] = upper_divider
                    new_trap.edges[1] = lower_divider
                    open_traps[adj_lines[2].bounds] = new_trap
                    open_traps[adj_lines[3].bounds] = new_trap
                    tm.traps.append(new_trap)
                    trap_id += 1
                    # Handle neighbors
                    prev_1.neighbors[2] = new_trap
                    prev_2.neighbors[2] = new_trap
                    new_trap.neighbors[0] = prev_1
                    new_trap.neighbors[1] = prev_2
                else:
                    # Close trapezoid to the left
                    open_traps.pop(seg_dict[nxt][0].bounds)
                    open_traps.pop(seg_dict[nxt][1].bounds)
            else:
                # The very last event point, end of environment boundary
                open_traps.pop(seg_dict[nxt][0].bounds)
                open_traps.pop(seg_dict[nxt][1].bounds)
        elif other_coords0[0] > nxt[0] and other_coords1[0] > nxt[0]:
            # Both adjacent lines are to the right of the sweep line
            s_line.append(seg_dict[nxt][0])
            s_line.append(seg_dict[nxt][1])
            if adj_lines is not None:
                if adj_lines[0][0] is not None and adj_lines[0][1] is not None and adj_lines[1][0] is not None and adj_lines[1][1] is not None:
                    upper_divider = sg.MultiLineString([(nxt, adj_lines[0])])
                    lower_divider = sg.MultiLineString([(nxt, adj_lines[1])])
                else:
                    # Create trapezoid to the right
                    new_trap = TrapMap.TrapNode(trap_id)
                    open_traps[seg_dict[nxt][0].bounds] = new_trap
                    open_traps[seg_dict[nxt][1].bounds] = new_trap
                    tm.traps.append(new_trap)
                    trap_id += 1
                    continue
                if not is_convex(obs, upper_divider, lower_divider):
                    # Close trapezoid to the left
                    prev = open_traps.pop(adj_lines[2].bounds)
                    open_traps.pop(adj_lines[3].bounds)
                    prev.edges[2] = upper_divider
                    prev.edges[3] = lower_divider
                    # Create two new trapezoids
                    new_trap1 = TrapMap.TrapNode(trap_id)
                    new_trap1.edges[0] = upper_divider
                    open_traps[adj_lines[2].bounds] = new_trap1
                    if is_upper_line(seg_dict[nxt][0], seg_dict[nxt][1]):
                        open_traps[seg_dict[nxt][0].bounds] = new_trap1
                    else:
                        open_traps[seg_dict[nxt][1].bounds] = new_trap1
                    tm.traps.append(new_trap1)
                    trap_id += 1
                    new_trap2 = TrapMap.TrapNode(trap_id)
                    new_trap2.edges[0] = lower_divider
                    open_traps[adj_lines[3].bounds] = new_trap2
                    if is_upper_line(seg_dict[nxt][0], seg_dict[nxt][1]):
                        open_traps[seg_dict[nxt][1].bounds] = new_trap2
                    else:
                        open_traps[seg_dict[nxt][0].bounds] = new_trap2
                    tm.traps.append(new_trap2)
                    trap_id += 1
                    # Handle neighbors
                    prev.neighbors[2] = new_trap1
                    prev.neighbors[3] = new_trap2
                    new_trap1.neighbors[0] = prev
                    new_trap2.neighbors[0] = prev
                else:
                    # Create trapezoid to the right
                    new_trap = TrapMap.TrapNode(trap_id)
                    open_traps[seg_dict[nxt][0].bounds] = new_trap
                    open_traps[seg_dict[nxt][1].bounds] = new_trap
                    tm.traps.append(new_trap)
                    trap_id += 1
            else:
                # The very first event point, start of environment boundary
                new_trap = TrapMap.TrapNode(trap_id)
                open_traps[seg_dict[nxt][0].bounds] = new_trap
                open_traps[seg_dict[nxt][1].bounds] = new_trap
                tm.traps.append(new_trap)
                trap_id += 1
        elif other_coords0[0] < nxt[0] < other_coords1[0]:
            # Adjacent line 0 is to the the left and adjacent line 1 is to the right of the sweep line
            s_line.remove(seg_dict[nxt][0])
            s_line.append(seg_dict[nxt][1])
            if ((other_coords0[1] > nxt[1] or other_coords1[1] > nxt[1]) and adj_lines[1][1] is not None) or \
                    ((other_coords0[1] < nxt[1] and other_coords1[1] < nxt[1]) and adj_lines[0][1] is None):
                lower_divider = sg.MultiLineString([(nxt, adj_lines[1])])
                # Close trapezoid to the left
                open_traps.pop(seg_dict[nxt][0].bounds)
                prev = open_traps.pop(adj_lines[3].bounds)
                prev.edges[2] = lower_divider
                # Create new trapezoid
                new_trap = TrapMap.TrapNode(trap_id)
                new_trap.edges[0] = lower_divider
                open_traps[seg_dict[nxt][1].bounds] = new_trap
                open_traps[adj_lines[3].bounds] = new_trap
                tm.traps.append(new_trap)
                trap_id += 1
                # Handle neighbors
                prev.neighbors[2] = new_trap
                new_trap.neighbors[0] = prev
            else:
                upper_divider = sg.MultiLineString([(nxt, adj_lines[0])])
                # Close trapezoid to the left
                open_traps.pop(seg_dict[nxt][0].bounds)
                prev = open_traps.pop(adj_lines[2].bounds)
                prev.edges[2] = upper_divider
                # Create new trapezoid
                new_trap = TrapMap.TrapNode(trap_id)
                new_trap.edges[0] = upper_divider
                open_traps[seg_dict[nxt][1].bounds] = new_trap
                open_traps[adj_lines[2].bounds] = new_trap
                tm.traps.append(new_trap)
                trap_id += 1
                # Handle neighbors
                prev.neighbors[2] = new_trap
                new_trap.neighbors[0] = prev
        else:
            # Adjacent line 1 is to the the left and adjacent line 0 is to the right of the sweep line
            s_line.remove(seg_dict[nxt][1])
            s_line.append(seg_dict[nxt][0])
            if((other_coords0[1] > nxt[1] or other_coords1[1] > nxt[1]) and adj_lines[1][1] is not None) or \
                    ((other_coords0[1] < nxt[1] and other_coords1[1] < nxt[1]) and adj_lines[0][1] is None):
                lower_divider = sg.MultiLineString([(nxt, adj_lines[1])])
                # Close trapezoid to the left
                open_traps.pop(seg_dict[nxt][1].bounds)
                prev = open_traps.pop(adj_lines[3].bounds)
                prev.edges[2] = lower_divider
                # Create new trapezoid
                new_trap = TrapMap.TrapNode(trap_id)
                new_trap.edges[0] = lower_divider
                open_traps[seg_dict[nxt][0].bounds] = new_trap
                open_traps[adj_lines[3].bounds] = new_trap
                tm.traps.append(new_trap)
                trap_id += 1
                # Handle neighbors
                prev.neighbors[2] = new_trap
                new_trap.neighbors[0] = prev
            else:
                upper_divider = sg.MultiLineString([(nxt, adj_lines[0])])
                # Close trapezoid to the left
                open_traps.pop(seg_dict[nxt][1].bounds)
                prev = open_traps.pop(adj_lines[2].bounds)
                prev.edges[2] = upper_divider
                # Create new trapezoid
                new_trap = TrapMap.TrapNode(trap_id)
                new_trap.edges[0] = upper_divider
                open_traps[seg_dict[nxt][0].bounds] = new_trap
                open_traps[adj_lines[2].bounds] = new_trap
                tm.traps.append(new_trap)
                trap_id += 1
                # Handle neighbors
                prev.neighbors[2] = new_trap
                new_trap.neighbors[0] = prev
    return tm
