import shapely.geometry as sg
import Trapezoidation
import RandDecomp
import RandMap
import PathFinding
import tkinter as tk
import math
import DiscConversion

class MotionPlanning():
    def __init__(self):
        # ************************************ Test Data ************************************
        self.ob1 = sg.MultiLineString(
            [((85, 490), (285, 350)), ((285, 350), (520, 412)), ((520, 412), (400, 632)), ((400, 632), (85, 490))])
        self.ob2 = sg.MultiLineString(
            [((389, 290), (755, 385)), ((755, 385), (564, 720)), ((564, 720), (600, 399)), ((600, 399), (389, 290))])
        self.env1 = sg.MultiLineString([((30, 500), (260, 40)), ((260, 40), (730, 60)), ((730, 60), (970, 375)),
                                   ((970, 375), (672, 889)), ((672, 889), (321, 940)), ((321, 940), (30, 500))])
        self.obstacle_set1 = [self.env1, self.ob1, self.ob2]
        self.effective_set1 = None

        self.ob3 = sg.MultiLineString([((466, 800), (500, 144)), ((500, 144), (752, 798)), ((752, 798), (466, 800))])
        self.env2 = sg.MultiLineString(
            [((22, 955), (381, 490)), ((381, 490), (26, 34)), ((26, 34), (960, 25)), ((960, 25), (726, 502)),
             ((726, 502), (975, 946)), ((975, 946), (22, 955))])
        self.obstacle_set2 = [self.env2, self.ob3]
        self.effective_set2 = None

        self.ob4 = sg.MultiLineString(
            [((100, 306), (105, 284)), ((105, 284), (782, 280)), ((782, 280), (795, 302)), ((795, 302), (100, 306))])
        self.ob5 = sg.MultiLineString(
            [((249, 610), (560, 415)), ((560, 415), (804, 457)), ((804, 457), (510, 666)), ((510, 666), (249, 610))])
        self.ob6 = sg.MultiLineString([((108, 766), (787, 764)), ((787, 764), (521, 911)), ((521, 911), (108, 766))])
        self.env3 = sg.MultiLineString(
            [((40, 975), (25, 289)), ((25, 289), (532, 26)), ((532, 26), (970, 304)), ((970, 304), (990, 974)),
             ((990, 974), (40, 975))])
        self.obstacle_set3 = [self.env3, self.ob4, self.ob5, self.ob6]
        self.effective_set3 = None
        # ***********************************************************************************
        self.edges = []
        self.levels = {}
        self.level_count = 0
        self.s_point = None
        self.t_point = None
        self.can_mark = False
        self.tm = None
        self.cm = None
        self.graph = None
        self.node_s = None
        self.node_t = None
        self.mode = None
        self.min_res = 50
        self.path_comps = []
        self.assume_disc = False
        # ************************************* Tkinter *************************************
        self.window = tk.Tk()
        self.window.bind("<Button 1>", self.mark_point)
        self.c = tk.Canvas(self.window, height=1000, width=1000)
        self.c.grid(row=0, column=0)
        self.settings_frame = tk.Frame(self.window)
        self.settings_frame.grid(row=2, column=0)
        self.var = tk.StringVar(self.window)
        self.var.trace("w", self.switch_up)
        self.drop = tk.OptionMenu(self.window, self.var, "set1", "set2", "set3")
        self.drop.grid(row=1, column=0)
        self.btn = tk.Button(self.settings_frame, text="Trapezoid", command=self.decomp_animation1)
        self.btn.grid(row=0, column=0)
        self.btn["state"] = "disabled"
        self.btn2 = tk.Button(self.settings_frame, text="Random", command=self.decomp_animation2)
        self.btn2.grid(row=0, column=3)
        self.btn2["state"] = "disabled"
        self.btn3 = tk.Button(self.settings_frame, text="Next Line", command=self.draw_line)
        self.btn3.grid(row=2, column=0)
        self.btn3["state"] = "disabled"
        self.btn4 = tk.Button(self.settings_frame, text="Get Path", command=self.draw_path)
        self.btn4.grid(row=2, column=3)
        self.btn4["state"] = "disabled"
        self.btn5 = tk.Button(self.settings_frame, text="Next Level", command=self.draw_cells)
        self.btn5.grid(row=2, column=1)
        self.btn5["state"] = "disabled"
        self.btn6 = tk.Button(self.settings_frame, text="Reset", command=self.reset_current)
        self.btn6.grid(row=0, column=1)
        self.btn6["state"] = "disabled"
        self.path_dist = tk.StringVar()
        self.path_dist.set(0)
        self.label = tk.Label(self.settings_frame, textvariable=self.path_dist)
        self.label.grid(row=3, column=1)
        self.label2 = tk.Label(self.settings_frame, text="Min Resolution:")
        self.label2.grid(row=1, column=0)
        self.label4 = tk.Label(self.settings_frame, text="Path Distance:")
        self.label4.grid(row=3, column=0)
        self.res_string = tk.StringVar()
        self.res_string.set(self.min_res)
        self.label3 = tk.Label(self.settings_frame, textvariable=self.res_string)
        self.label3.grid(row=1, column=1)
        self.field1 = tk.Entry(self.settings_frame)
        self.field1.grid(row=1, column=2)
        self.btn7 = tk.Button(self.settings_frame, text="Set", command=self.set_resolution)
        self.btn7.grid(row=1, column=3)
        self.btn7["state"] = "normal"
        self.btn8 = tk.Button(self.settings_frame, text="Mark Points", command=self.allow_marking)
        self.btn8.grid(row=2, column=2)
        self.btn8["state"] = "disabled"
        self.btn9 = tk.Button(self.settings_frame, text="Reset Path", command=self.reset_path)
        self.btn9.grid(row=0, column=2)
        self.btn9["state"] = "disabled"
        self.label5 = tk.Label(self.settings_frame, text="Disc Robot:")
        self.label5.grid(row=3, column=2)
        self.var2 = tk.IntVar()
        self.checkbox = tk.Checkbutton(self.settings_frame, variable=self.var2, onvalue=1, offvalue=0, command=self.disc_checked)
        self.checkbox.grid(row=3, column=3)
        self.window.mainloop()
        # ***********************************************************************************

    def set1(self):
        self.c.delete('all')
        for ob in self.obstacle_set1:
            for seg in ob:
                pt1 = seg.coords[0]
                pt2 = seg.coords[1]
                self.c.create_line(pt1[0], pt1[1], pt2[0], pt2[1], width=2)

    def set2(self):
        self.c.delete('all')
        for ob in self.obstacle_set2:
            for seg in ob:
                pt1 = seg.coords[0]
                pt2 = seg.coords[1]
                self.c.create_line(pt1[0], pt1[1], pt2[0], pt2[1], width=2)

    def set3(self):
        self.c.delete('all')
        for ob in self.obstacle_set3:
            for seg in ob:
                pt1 = seg.coords[0]
                pt2 = seg.coords[1]
                self.c.create_line(pt1[0], pt1[1], pt2[0], pt2[1], width=2)

    def switch_up(self, *args):
        self.btn["state"] = "normal"
        self.btn2["state"] = "normal"
        self.btn7["state"] = "normal"
        self.btn3["state"] = "disabled"
        self.btn5["state"] = "disabled"
        self.path_dist.set(0)
        self.btn9["state"] = "disabled"
        if self.var.get() == "set1":
            self.set1()
        elif self.var.get() == "set2":
            self.set2()
        else:
            self.set3()

    def reset_current(self):
        self.btn6["state"] = "disabled"
        self.switch_up()

    def decomp_animation1(self):
        self.c.delete('all')
        self.can_mark = False
        self.mode = 0
        self.btn["state"] = "disabled"
        self.btn2["state"] = "disabled"
        self.btn3["state"] = "normal"
        self.btn6["state"] = "normal"
        if self.var.get() == "set1":
            self.set1()
            if not self.assume_disc:
                self.effective_set1 = self.obstacle_set1
            else:
                self.effective_set1 = DiscConversion.convert(self.obstacle_set1)
            # Create dictionary of segments mapped to their endpoints
            seg_dict = Trapezoidation.extract_endpoints(self.effective_set1)
            # Sort all vertices of obstacles by x-coordinate and add to queue
            obstacle_points = Trapezoidation.merge_sort(list(seg_dict.keys()))
            # Environment object removed from set in order to check convexity in the decompose method
            mod_obs_set = self.obstacle_set1.copy()
            mod_obs_set.remove(self.env1)
        elif self.var.get() == "set2":
            self.set2()
            if not self.assume_disc:
                self.effective_set2 = self.obstacle_set2
            else:
                self.effective_set2 = DiscConversion.convert(self.obstacle_set2)
            # Create dictionary of segments mapped to their endpoints
            seg_dict = Trapezoidation.extract_endpoints(self.effective_set2)
            # Sort all vertices of obstacles by x-coordinate and add to queue
            obstacle_points = Trapezoidation.merge_sort(list(seg_dict.keys()))
            # Environment object removed from set in order to check convexity in the decompose method
            mod_obs_set = self.obstacle_set2.copy()
            mod_obs_set.remove(self.env2)
        else:
            self.set3()
            if not self.assume_disc:
                self.effective_set3 = self.obstacle_set3
            else:
                self.effective_set3 = DiscConversion.convert(self.obstacle_set3)
            # Create dictionary of segments mapped to their endpoints
            seg_dict = Trapezoidation.extract_endpoints(self.effective_set3)
            # Sort all vertices of obstacles by x-coordinate and add to queue
            obstacle_points = Trapezoidation.merge_sort(list(seg_dict.keys()))
            # Environment object removed from set in order to check convexity in the decompose method
            mod_obs_set = self.obstacle_set3.copy()
            mod_obs_set.remove(self.env3)
        # self.btn3.pack()
        # Create map of trapezoids and their associated edges
        self.tm = Trapezoidation.decompose(obstacle_points, mod_obs_set)
        self.edges.clear()
        for trap in self.tm.traps:
            for edge in trap.edges:
                if edge is not None and edge not in self.edges:
                    self.edges.append(edge)
        # Get adjacency list
        adj_list = self.tm.build_adjacency()
        # Build connectivity graph
        self.graph = PathFinding.build_graph(adj_list, 0)
        print("hello")

    def set_resolution(self):
        self.min_res = int(self.field1.get())
        self.res_string.set(self.min_res)
        self.label3.update_idletasks()
        self.field1.delete(0, 'end')

    def decomp_animation2(self):
        self.c.delete('all')
        self.levels.clear()
        self.can_mark = False
        self.level_count = 0
        self.mode = 1
        self.btn["state"] = "disabled"
        self.btn2["state"] = "disabled"
        self.btn5["state"] = "normal"
        self.btn7["state"] = "disabled"
        self.btn6["state"] = "normal"
        if self.var.get() == "set1":
            self.set1()
            if not self.assume_disc:
                self.effective_set1 = self.obstacle_set1
            else:
                self.effective_set1 = DiscConversion.convert(self.obstacle_set1)
            # Convert the obstacles representation to polygons
            poly_set = RandDecomp.strset_to_poly(self.effective_set1)
            # Get total environment space as a cell
            env_dim, env_pts = RandDecomp.get_bounding_space_cell(poly_set[0])
            root_cell = RandMap.CellNode(0, env_pts)
        elif self.var.get() == "set2":
            self.set2()
            if not self.assume_disc:
                self.effective_set2 = self.obstacle_set2
            else:
                self.effective_set2 = DiscConversion.convert(self.obstacle_set2)
            # Convert the obstacles representation to polygons
            poly_set = RandDecomp.strset_to_poly(self.effective_set2)
            # Get total environment space as a cell
            env_dim, env_pts = RandDecomp.get_bounding_space_cell(poly_set[0])
            root_cell = RandMap.CellNode(0, env_pts)
        else:
            self.set3()
            if not self.assume_disc:
                self.effective_set3 = self.obstacle_set3
            else:
                self.effective_set3 = DiscConversion.convert(self.obstacle_set3)
            # Convert the obstacles representation to polygons
            poly_set = RandDecomp.strset_to_poly(self.effective_set3)
            # Get total environment space as a cell
            env_dim, env_pts = RandDecomp.get_bounding_space_cell(poly_set[0])
            root_cell = RandMap.CellNode(0, env_pts)
        # self.btn5.pack()
        # Create map of cells
        self.cm = RandDecomp.decompose(root_cell, poly_set, env_dim, self.min_res)
        self.build_levels(self.cm.root)
        # Get adjacency list
        adj_list = self.cm.build_adjacency(self.cm.root)
        # Build connectivity graph
        self.graph = PathFinding.build_graph(adj_list, 1)

    def draw_line(self):
        if len(self.edges) != 0:
            curr = self.edges.pop(0)
            self.c.create_line(curr[0].bounds[0], curr[0].bounds[1], curr[0].bounds[2], curr[0].bounds[3])
            self.c.create_oval(curr.centroid.bounds[0] - 5, curr.centroid.bounds[1] - 5, curr.centroid.bounds[0] + 5,
                          curr.centroid.bounds[1] + 5, fill="#f4fc03")
            if len(self.edges) == 0:
                self.btn3["state"] = "disabled"
                self.s_point = None
                self.t_point = None
                self.btn8["state"] = "normal"

    def build_levels(self, curr_root):
        if curr_root.height not in self.levels.keys():
            self.levels[curr_root.height] = [curr_root]
        else:
            self.levels[curr_root.height].append(curr_root)
        for child in curr_root.children:
            if child is not None:
                self.build_levels(child)

    def draw_cells(self):
        if self.level_count < len(self.levels):
            curr_level = self.levels[self.level_count]
            self.level_count += 1
            for cell in curr_level:
                self.c.create_line(cell.coordinates[0][0], cell.coordinates[0][1], cell.coordinates[1][0],
                              cell.coordinates[1][1])
                self.c.create_line(cell.coordinates[1][0], cell.coordinates[1][1], cell.coordinates[3][0],
                              cell.coordinates[3][1])
                self.c.create_line(cell.coordinates[3][0], cell.coordinates[1][1], cell.coordinates[2][0],
                              cell.coordinates[2][1])
                self.c.create_line(cell.coordinates[2][0], cell.coordinates[2][1], cell.coordinates[0][0],
                              cell.coordinates[0][1])
                if cell.label == "empty":
                    self.c.create_oval(cell.centroid[0] - 5, cell.centroid[1] - 5, cell.centroid[0] + 5,
                                  cell.centroid[1] + 5, fill="#f4fc03")
            if self.level_count == len(self.levels):
                self.btn5["state"] = "disabled"
                self.s_point = None
                self.t_point = None
                self.btn8["state"] = "normal"

    def allow_marking(self):
        self.can_mark = True
        self.btn8["state"] = "disabled"

    def mark_point(self, eventorigin):
        curr = None
        if self.var.get() == "set1":
            curr = self.env1
        elif self.var.get() == "set2":
            curr = self.env2
        else:
            curr = self.env3
        curr_poly = RandDecomp.strset_to_poly([curr])
        if sg.Point(eventorigin.x, eventorigin.y).within(curr_poly[0]):
            if self.mode == 0:
                if self.s_point is None and self.can_mark:
                    self.s_point = (eventorigin.x, eventorigin.y)
                    x = self.c.create_oval(self.s_point[0] - 5, self.s_point[1] - 5, self.s_point[0] + 5, self.s_point[1] + 5,
                                       fill="#13fc03")
                    self.path_comps.append(x)
                    self.node_s = PathFinding.locate_cell_pts(sg.Point(self.s_point), 0, self.tm)
                    x = self.c.create_oval(self.node_s.bounds[0] - 5, self.node_s.bounds[1] - 5, self.node_s.bounds[0] + 5,
                                       self.node_s.bounds[1] + 5,
                                  fill="#03fcec")
                    self.path_comps.append(x)
                elif self.t_point is None and self.can_mark:
                    self.t_point = (eventorigin.x, eventorigin.y)
                    x = self.c.create_oval(self.t_point[0] - 5, self.t_point[1] - 5, self.t_point[0] + 5, self.t_point[1] + 5,
                                       fill="#fc0303")
                    self.path_comps.append(x)
                    self.node_t = PathFinding.locate_cell_pts(sg.Point(self.t_point), 0, self.tm)
                    x = self.c.create_oval(self.node_t.bounds[0] - 5, self.node_t.bounds[1] - 5, self.node_t.bounds[0] + 5,
                                       self.node_t.bounds[1] + 5,
                                  fill="#03fcec")
                    self.path_comps.append(x)
                    self.btn4["state"] = "normal"
                    self.can_mark = False
            else:
                if self.s_point is None and self.can_mark:
                    self.s_point = (eventorigin.x, eventorigin.y)
                    x = self.c.create_oval(self.s_point[0] - 5, self.s_point[1] - 5, self.s_point[0] + 5, self.s_point[1] + 5,
                                       fill="#13fc03")
                    self.path_comps.append(x)
                    self.node_s = PathFinding.locate_cell_pts(sg.Point(self.s_point), 1, self.cm)
                    x = self.c.create_oval(self.node_s.bounds[0] - 5, self.node_s.bounds[1] - 5, self.node_s.bounds[0] + 5,
                                       self.node_s.bounds[1] + 5,
                                  fill="#03fcec")
                    self.path_comps.append(x)
                elif self.t_point is None and self.can_mark:
                    self.t_point = (eventorigin.x, eventorigin.y)
                    x = self.c.create_oval(self.t_point[0] - 5, self.t_point[1] - 5, self.t_point[0] + 5, self.t_point[1] + 5,
                                       fill="#fc0303")
                    self.path_comps.append(x)
                    self.node_t = PathFinding.locate_cell_pts(sg.Point(self.t_point), 1, self.cm)
                    x = self.c.create_oval(self.node_t.bounds[0] - 5, self.node_t.bounds[1] - 5, self.node_t.bounds[0] + 5,
                                       self.node_t.bounds[1] + 5,
                                  fill="#03fcec")
                    self.path_comps.append(x)
                    self.btn4["state"] = "normal"
                    self.can_mark = False

    def draw_path(self):
        self.btn4["state"] = "disabled"
        path, dist = PathFinding.find_path(self.node_s, self.node_t, self.graph)
        prev_pt = None
        counter = len(path)
        for pt in path:
            if prev_pt is None:
                x = self.c.create_line(self.s_point[0], self.s_point[1], pt.bounds[0], pt.bounds[1],
                                   fill="#03fcec", width=2)
                prev_pt = pt
                self.path_comps.append(x)
                if len(path) == 1:
                    x = self.c.create_line(pt.bounds[0], pt.bounds[1], self.t_point[0], self.t_point[1],
                                           fill="#03fcec", width=2)
                    self.path_comps.append(x)
            else:
                x = self.c.create_line(prev_pt.bounds[0], prev_pt.bounds[1], pt.bounds[0], pt.bounds[1],
                                   fill="#03fcec", width=2)
                prev_pt = pt
                self.path_comps.append(x)
                if counter == 1:
                    x = self.c.create_line(pt.bounds[0], pt.bounds[1], self.t_point[0], self.t_point[1],
                                       fill="#03fcec", width=2)
                    self.path_comps.append(x)
            counter -= 1
        factor = 10.0 ** 3
        dist = math.trunc(dist * factor) / factor
        self.path_dist.set(dist)
        self.btn9["state"] = "normal"

    def reset_path(self):
        self.btn9["state"] = "disabled"
        for component in self.path_comps:
            self.c.delete(component)
        self.path_comps.clear()
        self.btn8["state"] = "normal"
        self.s_point = None
        self.t_point = None
        self.path_dist.set(0)

    def disc_checked(self):
        if self.var2.get() == 1:
            self.assume_disc = True
        else:
            self.assume_disc = False


MotionPlanning()
