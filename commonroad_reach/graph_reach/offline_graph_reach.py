import warnings

# from tests.python.deprecated.reach import ReachSet
#
# try:
#     # from commonroad_reach.reach.deprecated.reach import ReachsetForwardStep#, ReachSet
# except ImportError as e:
#     warnings.warn(str(e))

import pycrreach
from pycrreach import Polyhedron2VRep
from commonroad_dc.pycrcc import RectAABB
from commonroad.common.util import Interval
import math

from .utils import *
from typing import List
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import *



# from pruningPy.graph_reach.utils import *


class ReachSetGridOffline():
    def __init__(self, configuration):
        self.print_progress = True
        self.configuration = dict(configuration)
        self.time_slice: List[List[ReachSetNodeMultiGeneration]] = []
        self.reached_velocities_minmax_list = [[]]
        self.velocity_bounds = [[(Interval(0,0), Interval(0,0))]] # list of list of tuples with vx,vy intervals
        self.reach_rect_grid: List[np.ndarray] = [np.array([[0]])]  # assigns to grid coordinates the element id in time_slice
        self.reach_rect_grid_coordinate_list: List[List[tuple]] = [[0]]  # assigns to list element of time_slice the grid coordinates

        param_copy = {
            'dt': configuration['d_t'],
            'min_speed_x': configuration['min_speed_x'],
            'max_speed_x': configuration['max_speed_x'],
            'a_max_x': configuration['a_max_x'],
            'grid_x': configuration['grid_x'],
            'min_speed_y': configuration['min_speed_y'],
            'max_speed_y': configuration['max_speed_y'],
            'a_max_y': configuration['a_max_y'],
            'grid_y': configuration['grid_y'],
            'max_radius': configuration['max_radius'],
            'initial_time': configuration['initial_time_idx']
        }

        #reference coordinates for grid grid_reference[time_step][poly_index]
        self.grid_reference_lo = [(0,0)]
        self.grid_reference_hi = [(0,0)]
        self.reach_set_rects: List[List[pycrreach.AARectDouble]] = list()

        try:
            param_copy['bounding_boxes'] = configuration['bounding_boxes']
        except:
            pass

        try:
            param_copy['initial_time'] = configuration['initial_time']
        except:
            pass

        self.forward_step = ReachsetForwardStepUniformGrid(**param_copy)

        poly1 = pycrreach.Polyhedron2VRep()
        poly1.add_point(configuration['initial_state'][0][0],
                        configuration['initial_state'][0][1])
        poly2 = pycrreach.Polyhedron2VRep()
        poly2.add_point(configuration['initial_state'][1][0],
                        configuration['initial_state'][1][1])

        if 'task_specification' not in configuration:
            self.root_node = ReachSetNodeMultiGeneration(poly1, poly2,
                                                         configuration['initial_time_idx'])
        else:
            raise Exception

        self.time_slice.append([self.root_node])

    def compute_next_time_step(self):
        if 'task_specification' not in self.configuration:
            forward_reach_set,\
            x_ref_lo,\
            y_ref_lo,\
            x_ref_hi,\
            y_ref_hi,\
            velocities_minmax_tmp,\
            velocity_bounds_tmp,\
            reach_rect_grid,\
            reach_rect_grid_coordinate_list,\
            reach_aarect_double_after_split\
                = self.forward_step.forward_step(self.time_slice[-1], self.configuration)
            self.time_slice.append(forward_reach_set)
            self.grid_reference_lo.append((x_ref_lo, y_ref_lo))
            self.grid_reference_hi.append((x_ref_hi, y_ref_hi))
            self.reach_rect_grid.append(reach_rect_grid)
            self.reach_rect_grid_coordinate_list.append(reach_rect_grid_coordinate_list)
            self.reach_set_rects.append(reach_aarect_double_after_split)
            self.reached_velocities_minmax_list.append(velocities_minmax_tmp)
            self.velocity_bounds.append(velocity_bounds_tmp)
        else:
            raise Exception

    def compute_next_time_steps(self, n):
        for i in range(n):
            if self.print_progress:
                print('<ReachSetUniformGrid/compute_next_time_steps>: time_step', len(self.time_slice)-1, 'of', n-1)
            self.compute_next_time_step()

    def compute_grandchildren(self, delta_timestep_list: Union[int,List[int]], kamms_circle: bool = True):
        def apply_kamms_circle(i_t_current, id_node, node: ReachSetNodeMultiGeneration, radius_squared: float):
            time_slice_forward = self.time_slice[i_t+delta_timesteps]
            # compute reachable set, considering kamms circle:
            int_vx, int_vy = self.velocity_bounds[i_t_current][id_node]
            x_min_prop = int_vx.start * delta_nt + node.x_min()
            y_min_prop = int_vy.start * delta_nt + node.y_min()
            x_max_prop = int_vx.end * delta_nt + node.x_max()
            y_max_prop = int_vy.end * delta_nt + node.y_max()
            rect = RectAABB((x_max_prop - x_min_prop)/2, (y_max_prop - y_min_prop)/2, (x_max_prop + x_min_prop)/2, (y_max_prop + y_min_prop)/2)
            # plt.ion()
            # plt.figure()
            grandchilds = node.grandchild_nodes[delta_timesteps - 1]
            delete = list()
            for child_id in grandchilds:
                child = time_slice_forward[child_id]
                x_all = [child.x_min(), child.x_max()]
                y_all = [child.y_min(), child.y_max()]

                keep = False

                for x in x_all:
                    for y in y_all:
                        if np.less_equal(rect.sq_distance_to_point(x,y), radius_squared+0.0001):
                            keep = True
                            # draw_object(self.reach_set_rects[i_t_current+delta_timesteps-1][child_id], draw_params={'reach':{'aarect':{'facecolor': 'g'}}})
                    if keep:
                        break
                    if keep==False:
                        # draw_object(self.reach_set_rects[i_t_current + delta_timesteps-1][child_id],
                        #             draw_params={'reach': {'aarect': {'facecolor': 'r'}}})
                        delete.append(child_id)

            node.grandchild_nodes[delta_timesteps - 1] = list(set(grandchilds)-set(delete))

            # draw_object(rect,draw_params={'collision':{'zorder': 30}})
            # plt.autoscale()
            # plt.close()


        if not hasattr(self, '_last_grandchildren_computation'):
            self._last_grandchildren_computation = 0

        if type(delta_timestep_list) is int:
            delta_timestep_list = [delta_timestep_list]

        for delta_timesteps in delta_timestep_list:
            delta_timesteps += 1
            delta_nt = self.forward_step.dt * delta_timesteps
            # reachable sets for input and new delta_t
            reach_n_u_x = pycrreach.create_reach_u(delta_nt, self.configuration['a_max_x'])
            reach_n_u_y = pycrreach.create_reach_u(delta_nt, self.configuration['a_max_y'])

            for i_t, time_slice_current in enumerate(self.time_slice[:-delta_timesteps]):
                if i_t >= self._last_grandchildren_computation:
                    self.forward_step.forward_multiple_steps(time_slice_current, self.time_slice[i_t+delta_timesteps], self.reach_set_rects[i_t + delta_timesteps -1], delta_timesteps, reach_n_u_x, reach_n_u_y)
                    # self._last_grandchildren_computation += 1

                if kamms_circle == True and i_t>=3:
                    radius_squared = (max(self.configuration['a_max_x'], self.configuration['a_max_y']) * 0.5 * (delta_nt ** 2))**2
                    for id_node, node in enumerate(time_slice_current):
                        apply_kamms_circle(i_t, id_node, node, radius_squared)



class ReachsetForwardStep:

    def __init__(self, dt,
                 min_speed_x, max_speed_x, a_max_x, grid_x,
                 min_speed_y, max_speed_y, a_max_y, grid_y,
                 max_radius,
                 reach_u_x=None, reach_u_y=None,
                 bounding_boxes=None, initial_time=0):
        self.dt = dt
        self.min_speed_x = min_speed_x
        self.max_speed_x = max_speed_x
        self.a_max_x = a_max_x
        self.grid_x = grid_x
        self.min_speed_y = min_speed_y
        self.max_speed_y = max_speed_y
        self.a_max_y = a_max_y
        self.grid_y = grid_y
        self.max_radius = max_radius
        if not reach_u_x:
            self.reach_u_x = pycrreach.create_reach_u(dt, a_max_x)
        else:
            self.reach_u_x = reach_u_x
        if not reach_u_y:
            self.reach_u_y = pycrreach.create_reach_u(dt, a_max_y)
        else:
            self.reach_u_y = reach_u_y
        self.bounding_boxes = bounding_boxes
        self.initial_time = initial_time

    def forward_step(self, reach_boxes_last, configuration):
        """
        Computes the set of base sets for the next time step as described in the
        submitted paper.

        :param reach_boxes_last:
        :param configuration:
        :return: created reachboxes at time current_time+1
        """
        if len(reach_boxes_last) < 1:
            return []

        current_time = reach_boxes_last[0].time_idx
        try:
            self.bounding_boxes = configuration['bounding_boxes']
        except Exception:
            pass

        # propagate
        poly_x_q, poly_y_q = self.propagate(reach_boxes_last, current_time)

        reach_aarect_double_after_split = self.next_drivable_area(
            current_time, configuration, poly_x_q, poly_y_q)

        reach_boxes_next_step = self.next_reachboxes_and_connect_graph(
            reach_boxes_last, reach_aarect_double_after_split, poly_x_q, poly_y_q)
        return reach_boxes_next_step

    def next_drivable_area(self, current_time, configuration, poly_x_q, poly_y_q):
        # project and partition
        if configuration['coordinates'] == 'curvilinear':
            reach_aarect_double_after_split_curved = self.project_and_partition(
                poly_x_q, poly_y_q,
                configuration['collision_checker_curvilinear'], current_time + 1)
            reach_aarect_double_after_split = (
                split_cartesian_aa_rect_in_curved_coordinates(
                    configuration,
                    reach_aarect_double_after_split_curved,
                    configuration['collision_checker_world'],
                    current_time + 1)[0])
        elif configuration['coordinates'] == 'cartesian':
            reach_aarect_double_after_split = self.project_and_partition(
                poly_x_q, poly_y_q,
                configuration['collision_checker_world'], current_time + 1)
        else:
            raise Exception()

        # adapt to grid
        # TODO: condition reach_aarect_double_after_split
        return reach_aarect_double_after_split

    def next_reachboxes_and_connect_graph(self, reach_boxes_last, reach_aarect_double_after_split, poly_x_q, poly_y_q):
        # assign velocities to position regions
        poly_x_new, poly_y_new, parents_new = (
            self.create_basesets_list_from_position_region(
                poly_x_q, poly_y_q, reach_aarect_double_after_split))

        # add nodes and edges to graph of reachable regions
        reach_boxes_next_step = self.create_reachboxes_and_connect_graph(
            reach_boxes_last, poly_x_new, poly_y_new, parents_new)

        return reach_boxes_next_step

    def propagate(self, reachboxes, current_time):

        def propagate_x(poly_x):
            x = poly_x.copy()
            x.linear_map(1, self.dt, 0, 1)
            x = x.minkowski_sum(self.reach_u_x)
            x.intersect_halfspace(0, 1, self.max_speed_x)
            x.intersect_halfspace(0, -1, -self.min_speed_x)
            # TODO: ICS not tested
            if self.bounding_boxes:
                for time_ics in range(current_time + 1,
                                      self.initial_time + len(
                                          self.bounding_boxes)):
                    ics = ICSHalfspace(self.a_max_x, (
                    time_ics - (current_time + 1)) * self.dt,
                                       (self.bounding_boxes[
                                            time_ics - self.initial_time]['x1'],
                                        self.bounding_boxes[
                                            time_ics - self.initial_time][
                                            'x2']))
                    x.intersect_halfspace(ics[0][0], ics[0][1], ics[0][2])
                    x.intersect_halfspace(ics[1][0], ics[1][1], ics[1][2])
            return x

        def propagate_y(poly_y):
            y = poly_y.copy()
            y.linear_map(1, self.dt, 0, 1)
            y = y.minkowski_sum(self.reach_u_y)
            y.intersect_halfspace(0, 1, self.max_speed_y)
            y.intersect_halfspace(0, -1, -self.min_speed_y)
            # TODO: ICS not tested
            if self.bounding_boxes:
                for time_ics in range(current_time + 1,
                                      self.initial_time + len(
                                          self.bounding_boxes)):
                    ics = ICSHalfspace(self.a_max_y, (
                    time_ics - (current_time + 1)) * self.dt,
                                       (self.bounding_boxes[
                                            time_ics - self.initial_time]['y1'],
                                        self.bounding_boxes[
                                            time_ics - self.initial_time][
                                            'y2']))
                    y.intersect_halfspace(ics[0][0], ics[0][1], ics[0][2])
                    y.intersect_halfspace(ics[1][0], ics[1][1], ics[1][2])
            return y

        poly_x_q = [propagate_x(p.reach_x) for p in reachboxes]
        poly_y_q = [propagate_y(p.reach_y) for p in reachboxes]
        return poly_x_q, poly_y_q

    def project_and_partition(self, poly_x_q, poly_y_q, collision_checker,
                              current_time):
        x_c, y_c = self.find_bottom_left_of_reach_set_projection(
            poly_x_q, poly_y_q)

        # discretize
        reach_aarect_int_before_merge = (
            self.create_reach_aarect_int_from_reach_set_ignore_empty(
                poly_x_q, poly_y_q, x_c, y_c))

        # merge
        reach_aarect_int_after_merge = pycrreach.merge_and_split_aarect_int(
            reach_aarect_int_before_merge)

        # undiscretize
        reach_aarect_double_after_merge = self.undiscretize(
            reach_aarect_int_after_merge, x_c, y_c)

        # split
        reach_aarect_double_after_split = pycrreach.split_aarect_double(
            collision_checker, reach_aarect_double_after_merge,
            current_time, self.max_radius)[0]

        return reach_aarect_double_after_split

    def create_baseset_from_position_region(self, adj_pair_list, poly_x_q,
                                            poly_y_q, reach_aa_rect):
        """Creates two polyhedrons for the reachable position/speed set in x-
        and y-direction

        Args:
            adj_pair_list: list of tuples; the first element of the tuple
                corresponds to the index in the list poly_x_q and poly_y_q; the
                second element of all tuples is the same and corresponds to the
                index of reach_aa_rect in the calling function.
            poly_x_q: List of pycrreach.Polyhedron2VRep
            poly_y_q: List of pycrreach.Polyhedron2VRep
            reach_aa_rect: pycrreach.AARectDouble
        """
        # Returns empty poly_x, poly_y, parents if not reachable
        poly_x = pycrreach.Polyhedron2VRep()
        poly_y = pycrreach.Polyhedron2VRep()
        parents = list()
        for adj in adj_pair_list:
            parent_idx = adj[0]
            tmp_x = poly_x_q[parent_idx].copy()
            tmp_x.intersect_halfspace(1, 0, reach_aa_rect.x_hi)
            tmp_x.intersect_halfspace(-1, 0, -reach_aa_rect.x_lo)

            tmp_y = poly_y_q[parent_idx].copy()
            tmp_y.intersect_halfspace(1, 0, reach_aa_rect.y_hi)
            tmp_y.intersect_halfspace(-1, 0, -reach_aa_rect.y_lo)

            if not (tmp_x.is_empty() or tmp_y.is_empty()):
                poly_x.add_polyhedron(tmp_x)
                poly_y.add_polyhedron(tmp_y)
                parents.append(parent_idx)
        return poly_x, poly_y, parents

    def create_basesets_list_from_position_region(self, poly_x_q, poly_y_q,
                                                  reach_aarect_regions):
        """Creates two polyhedrons for the reachable position/speed set in x-
        and y-direction of each position region of reach_aarect_regions and the
        indices in poly_x_q/poly_x_q which contributes to the reachable
        velocities.

        """
        adj = self.determine_position_overlap(poly_x_q, poly_y_q,
                                              reach_aarect_regions)
        adj.sort(key=lambda x: x[1])
        slice_begin = 0
        slice_end = 0
        parents_new = list()
        poly_x_new = list()
        poly_y_new = list()
        while slice_begin < len(adj):
            slice_end = slice_begin
            while (slice_end + 1 < len(adj) and
                   adj[slice_end + 1][1] == adj[slice_begin][1]):
                slice_end = slice_end + 1
            tmp_x, tmp_y, tmp_parent = self.create_baseset_from_position_region(
                adj[slice_begin:slice_end + 1], poly_x_q, poly_y_q,
                reach_aarect_regions[adj[slice_begin][1]])
            poly_x_new.append(tmp_x)
            poly_y_new.append(tmp_y)
            parents_new.append(tmp_parent)
            slice_begin = slice_end + 1
        return poly_x_new, poly_y_new, parents_new

    def create_reachboxes_and_connect_graph(
        self, last_reachboxes, poly_x, poly_y, parents):
        time_idx = last_reachboxes[-1].time_idx + 1
        new_reachset = list()
        for i in range(len(poly_x)): # == len(poly_y) == len(parents)
            if parents[i]: # == (poly_x[i].is_empty() or poly_y[i].is_empty())
                reach_box = ReachSetNode(poly_x[i], poly_y[i], time_idx)
                #reach_box.setBoundFromReachset()
                for p in parents[i]:
                    reach_box.add_parent(last_reachboxes[p])
                    last_reachboxes[p].add_child(reach_box)
                new_reachset.append(reach_box)
        return new_reachset

    def determine_position_overlap(self, poly_x_q, poly_y_q,
                                   aa_rect_double_list):

        def project_basesets_into_position_domain(poly_x_q, poly_y_q):
            projected_aarect_double = []
            for r in range(0, len(poly_x_q)):
                if poly_x_q[r].is_empty() or poly_y_q[r].is_empty():
                    #print("Warning: dummy aa rect inserted")
                    projected_aarect_double.append(None)
                else:
                    projected_aarect_double.append(
                        pycrreach.AARectDouble(poly_x_q[r].min_x(),
                                               poly_x_q[r].max_x(),
                                               poly_y_q[r].min_x(),
                                               poly_y_q[r].max_x()))
            return projected_aarect_double

        def add_dummy_rectangles(projected_aarect_double, aa_rect_double_list):
            """
            Add for each "None" in the projected_aarect_double list a
            pycrreach.AARectDouble object which is guaranteed to do not
            overlap with any element of aa_rect_double_list.
            """
            x_min = aa_rect_double_list[0].x_lo
            y_min = aa_rect_double_list[0].y_lo
            for rect in aa_rect_double_list:
                x_min = min(x_min, rect.x_lo)
                y_min = min(y_min, rect.y_lo)

            for i in range(len(projected_aarect_double)):
                if projected_aarect_double[i] is None:
                    projected_aarect_double[i] = pycrreach.AARectDouble(
                        x_min - 2.0, x_min - 1.0, y_min - 2.0, y_min - 1.0)

        if len(aa_rect_double_list) < 1:
            return []

        projected_aarect_double = project_basesets_into_position_domain(
            poly_x_q, poly_y_q)

        add_dummy_rectangles(projected_aarect_double, aa_rect_double_list)

        adj = pycrreach.overlap_aarect_double(projected_aarect_double,
                                              aa_rect_double_list)
        return adj

    def find_bottom_left_of_reach_set_projection(self, poly_x_q, poly_y_q):
        x_c = poly_x_q[0].min_x()
        for p in poly_x_q:
            x_c = x_c if x_c <= p.min_x() else p.min_x()

        y_c = poly_y_q[0].min_x()
        for p in poly_y_q:
            y_c = y_c if y_c <= p.min_x() else p.min_x()

        return x_c, y_c

    def create_reach_aarect_int_from_reach_set_ignore_empty(
            self, poly_x_q, poly_y_q, x_c, y_c):
        """
        Create pycrreach.AARect for each reach base set. Ignore a base set if
        it is empty.

        :param poly_x_q:
        :param poly_y_q:
        :param x_c:
        :param y_c:
        :return:
        """
        reach_aarect_int_before_merge = []
        for r in range(0, len(poly_x_q)):
            if not (poly_x_q[r].is_empty() or poly_y_q[r].is_empty()):
                x_min = poly_x_q[r].min_x()
                x_max = poly_x_q[r].max_x()
                y_min = poly_y_q[r].min_x()
                y_max = poly_y_q[r].max_x()
                reach_aarect_int_before_merge.append(
                    pycrreach.AARectInt(math.floor((x_min-x_c)/self.grid_x),
                                           math.ceil((x_max-x_c)/self.grid_x),
                                           math.floor((y_min-y_c)/self.grid_y),
                                           math.ceil((y_max-y_c)/self.grid_y)))
        return reach_aarect_int_before_merge

    def undiscretize(self, reach_aarect_int_list, x_c, y_c):
        reach_aarect_double_after_merge = []
        for r in reach_aarect_int_list:
            reach_aarect_double_after_merge.append(pycrreach.AARectDouble(
                r.x_lo*self.grid_x+x_c,
                r.x_hi*self.grid_x+x_c,
                r.y_lo*self.grid_y+y_c,
                r.y_hi*self.grid_y+y_c))
        return reach_aarect_double_after_merge

class ReachsetForwardStepUniformGrid(ReachsetForwardStep):
    def __init__(self, **kwargs):
        ReachsetForwardStep.__init__(self, **kwargs)


    def forward_step(self, reach_boxes_last, configuration):

        if len(reach_boxes_last) < 1:
            return []

        self.current_time = reach_boxes_last[0].time_idx
        try:
            self.bounding_boxes = configuration['bounding_boxes']
        except Exception:
            pass

        # propagate
        poly_x_q, poly_y_q = self.propagate(reach_boxes_last, self.current_time)

        # project and partition
        if configuration['coordinates'] == 'cartesian':
            reach_aarect_double_after_split = self.project_and_partition(
                poly_x_q, poly_y_q,
                configuration['collision_checker_world'], self.current_time + 1)
        else:
            raise Exception()

        reach_aarect_double_after_split = self.intersects_kamms_circle(reach_aarect_double_after_split)

        # adapt to grid
        # TODO: condition reach_aarect_double_after_split
        reach_rect_grid, reach_rect_grid_coordinate_list = self.assign_rects_to_grid(reach_aarect_double_after_split, poly_x_q, poly_y_q)

        # assign velocities to position regions
        poly_x_new, poly_y_new, parents_new, velocities_minmax, velocity_bounds, reach_aarect_double_reduced =\
            self.create_basesets_list_from_position_region_clean_redundant(
                poly_x_q, poly_y_q, reach_aarect_double_after_split, reach_rect_grid, reach_rect_grid_coordinate_list)

        # find reference points for storing (x_ref_hi changed due to intersection with polygons)
        x_ref_lo, y_ref_lo, x_ref_hi, y_ref_hi = self.find_bottom_left_top_right_of_reach_set_projection(poly_x_new, poly_y_new)

        # add nodes and edges to graph of reachable regions
        reach_boxes_next_step = self.create_reachboxes_and_connect_graph_multigeneration(
            reach_boxes_last, poly_x_new, poly_y_new, parents_new)

        return reach_boxes_next_step, x_ref_lo, y_ref_lo, x_ref_hi, y_ref_hi, velocities_minmax, velocity_bounds, reach_rect_grid, reach_rect_grid_coordinate_list, reach_aarect_double_reduced

    def intersects_kamms_circle(self, reach_aarect):
        i_t = self.current_time +1
        radius = 0.5 * max(self.a_max_x,self.a_max_y) * (self.dt*i_t) ** 2 + 0.01
        collision_circle = pycrcc.Circle(radius+0.0,0.0,0.0)

        delete_list = list()
        for i_rect, rect in enumerate(reach_aarect):
            collision_rect = pycrcc.RectAABB((rect.x_hi-rect.x_lo)/2, (rect.y_hi-rect.y_lo)/2, (rect.x_hi+rect.x_lo)/2, (rect.y_hi+rect.y_lo)/2)
            if not collision_circle.collide(collision_rect):
                delete_list.append(i_rect)

        print(len(reach_aarect),len(delete_list))
        for i_del in reversed(delete_list):
            del reach_aarect[i_del]

        return reach_aarect

    def create_reachboxes_and_connect_graph_multigeneration(self, last_reachboxes, poly_x, poly_y, parents):
        time_idx = last_reachboxes[-1].time_idx + 1
        new_reachset = list()
        for i in range(len(poly_x)): # == len(poly_y) == len(parents)
            if parents[i]: # == (poly_x[i].is_empty() or poly_y[i].is_empty())
                reach_box = ReachSetNodeMultiGeneration(poly_x[i], poly_y[i], time_idx)
                #reach_box.setBoundFromReachset()
                for p in parents[i]:
                    reach_box.add_parent(last_reachboxes[p])
                    last_reachboxes[p].add_child(reach_box)
                new_reachset.append(reach_box)
        return new_reachset

    def assign_rects_to_grid(self, reach_aarect_regions: List[pycrreach.AARectDouble], poly_x_new, poly_y_new) -> (np.ndarray, List[tuple]):
        """
        Assigns each rect to a coordinate [ix,iy] in a regular grid for quick access
        :param reach_aarect_regions:
        :param poly_x_new:
        :param poly_y_new:
        :return:
        """
        x_ref_lo, y_ref_lo, x_ref_hi, y_ref_hi = self.find_bottom_left_top_right_of_reach_set_projection(poly_x_new,
                                                                                                         poly_y_new)

        nx = int(np.ceil((x_ref_hi - x_ref_lo) / self.grid_x - 0.00001))
        ny = int(np.ceil((y_ref_hi - y_ref_lo) / self.grid_y - 0.00001))

        reach_rect_grid = np.empty([nx,ny], dtype=list)
        reach_rect_grid_coordinate_list = list()
        for i_rect, rect in enumerate(reach_aarect_regions):
            ix = round((rect.x_lo - x_ref_lo) / self.grid_x)
            iy = round((rect.y_lo - y_ref_lo) / self.grid_y)

            if reach_rect_grid[ix,iy] is not None:
                reach_rect_grid[ix, iy].append(i_rect)
            else:
                reach_rect_grid[ix,iy] = [i_rect]

            reach_rect_grid_coordinate_list.append((ix,iy))
        return reach_rect_grid, reach_rect_grid_coordinate_list


    def create_basesets_list_from_position_region_clean_redundant(self, poly_x_q, poly_y_q,
                                                  reach_aarect_regions, reach_rect_grid, reach_rect_grid_coordinate_list):
        """Creates two polyhedrons for the reachable position/speed set in x-
        and y-direction of each position region of reach_aarect_regions and the
        indices in poly_x_q/poly_x_q which contributes to the reachable
        velocities.

        """

        def is_same_rect(rect_A: pycrreach.AARectDouble, rect_B: pycrreach.AARectDouble):
            if rect_A.x_lo == rect_B.x_lo and \
            rect_A.y_lo == rect_B.y_lo and \
            rect_A.x_hi == rect_B.x_hi and \
            rect_A.y_hi == rect_B.y_hi:
                return True
            else:
                return False

        def is_same_polyhedron(poly_A, poly_B):
            # return true
            verts_A: np.ndarray = poly_A.get_vertices()
            verts_A.sort(axis=0)
            verts_B: np.ndarray = poly_B.get_vertices()
            verts_B.sort(axis=0)
            if np.array_equal(verts_A, verts_B):
                return True
            else:
                print('not same poly')
                return False

        adj = self.determine_position_overlap(poly_x_q, poly_y_q,
                                              reach_aarect_regions)
        adj.sort(key=lambda x: x[1]) # (parent_id, child_id)
        slice_begin = 0
        slice_end = 0
        parents_new = list()
        poly_x_new = list()
        poly_y_new = list()
        reached_velocities_minmax = list()
        velocity_bounds = list()
        reach_aarect_regions_new = list()
        redundancies = [list() for _ in range(0,len(reach_aarect_regions))] # for each index of reach_aarect_regions, this list will contains the index of the redundant rect
        erase_rect_and_poly_indices = list()
        while slice_begin < len(adj):
            # add_new_poly = True
            slice_end = slice_begin
            while (slice_end + 1 < len(adj) and
                   adj[slice_end + 1][1] == adj[slice_begin][1]):
                slice_end = slice_end + 1
            tmp_poly_x, tmp_poly_y, tmp_parent, velocities_minmax_tmp, velocity_bounds_tmp = self.create_baseset_from_position_region_get_velocity(
                adj[slice_begin:slice_end + 1], poly_x_q, poly_y_q,
                reach_aarect_regions[adj[slice_begin][1]])

            current_rect = reach_aarect_regions[adj[slice_begin][1]]
            if slice_begin > 0:
                #check whether two rects + corresp. polyhedrons are equal
                current_id = len(poly_x_new)

                # for i, rect_i in enumerate(reach_aarect_regions_new): #search in all rects that have been used to far
                rect_ids_tmp = reach_rect_grid[reach_rect_grid_coordinate_list[current_id]]
                # plt.figure()
                # for r in rect_ids_tmp:
                #     draw_object(reach_aarect_regions[r])
                # plt.show()
                for id in rect_ids_tmp:
                    if id < current_id and id not in erase_rect_and_poly_indices and is_same_rect(current_rect, reach_aarect_regions[id]) and is_same_polyhedron(tmp_poly_x, poly_x_new[id]):# and is_same_polyhedron(tmp_poly_y, poly_y_new[id]):
                        redundancies[id].append(current_id)
                        erase_rect_and_poly_indices.append(current_id)
                        # print(id)
                        #parents_new[id] += tmp_parent
                        # add_new_poly = False
                        break
                        # except:
                        #     iii=1
            poly_x_new.append(tmp_poly_x)
            poly_y_new.append(tmp_poly_y)
            reached_velocities_minmax.append(velocities_minmax_tmp) # containts in the end list of dict of min/max velocities for each parent list(dict(parent_id: [[vx_min, vx_max],[vx_min, vx_max]]))
            velocity_bounds.append(velocity_bounds_tmp)
            parents_new.append(tmp_parent)
            reach_aarect_regions_new.append(current_rect)
            slice_begin = slice_end + 1

        for i in sorted(erase_rect_and_poly_indices, reverse=True):
            # print(len(poly_x_new))
            # print(i)
            del poly_x_new[i]
            del poly_y_new[i]
            del reached_velocities_minmax[i]
            del velocity_bounds[i]
            del parents_new[i]
            del reach_aarect_regions_new[i]
        print('len_aft', len(reach_aarect_regions_new))
        return poly_x_new, poly_y_new, parents_new, reached_velocities_minmax, velocity_bounds, reach_aarect_regions_new

    def create_baseset_from_position_region_get_velocity(self, adj_pair_list, poly_x_q,
                                            poly_y_q, reach_aa_rect):

        """Creates two polyhedrons for the reachable position/speed set in x-
        and y-direction

        Args:
            adj_pair_list: list of tuples; the first element of the tuple
                corresponds to the index in the list poly_x_q and poly_y_q; the
                second element of all tuples is the same and corresponds to the
                index of reach_aa_rect in the calling function.
            poly_x_q: List of pycrreach.Polyhedron2VRep
            poly_y_q: List of pycrreach.Polyhedron2VRep
            reach_aa_rect: pycrreach.AARectDouble
        """
        # Returns empty poly_x, poly_y, parents if not reachable
        poly_x = pycrreach.Polyhedron2VRep()
        poly_y = pycrreach.Polyhedron2VRep()
        parents = list()
        velocities_minmax = dict()

        for adj in adj_pair_list:
            parent_idx = adj[0]
            tmp_x = poly_x_q[parent_idx].copy()
            tmp_x.intersect_halfspace(1, 0, reach_aa_rect.x_hi)
            tmp_x.intersect_halfspace(-1, 0, -reach_aa_rect.x_lo)

            tmp_y = poly_y_q[parent_idx].copy()
            tmp_y.intersect_halfspace(1, 0, reach_aa_rect.y_hi)
            tmp_y.intersect_halfspace(-1, 0, -reach_aa_rect.y_lo)

            if not (tmp_x.is_empty() or tmp_y.is_empty()):
                poly_x.add_polyhedron(tmp_x)
                poly_y.add_polyhedron(tmp_y)
                parents.append(parent_idx)
                # save min/max velocity values
                velocities_minmax.update({parent_idx: [Interval(tmp_x.min_y(), tmp_x.max_y()), Interval(tmp_y.min_y(), tmp_y.max_y())]})
            else:
                print('empty!!')

        velocity_bounds = ( Interval(poly_x.min_y(), poly_x.max_y()),
                            Interval(poly_y.min_y(), poly_y.max_y()) )
        # # FOR TESTING::
        # for adj in adj_pair_list:
        #     parent_idx = adj[0]
        #     tmp_x = poly_x_q[parent_idx].copy()
        #     tmp_x.intersect_halfspace(1, 0, reach_aa_rect.x_hi)
        #     tmp_x.intersect_halfspace(-1, 0, -reach_aa_rect.x_lo)
        #
        #     tmp_y = poly_y_q[parent_idx].copy()
        #     tmp_y.intersect_halfspace(1, 0, reach_aa_rect.y_hi)
        #     tmp_y.intersect_halfspace(-1, 0, -reach_aa_rect.y_lo)
        #
        #     if not (tmp_x.is_empty() or tmp_y.is_empty()):
        #         plt.close('all')
        #
        #         draw_object(poly_x)
        #         draw_object(tmp_x)
        #         plt.show()
        #
        #         plt.figure()
        #         draw_object(poly_y)
        #         draw_object(tmp_y)
        #         plt.show()
        #         iiii=1

        return poly_x, poly_y, parents, velocities_minmax, velocity_bounds

    def create_reach_aarect_grid_int_from_reach_set_ignore_empty(
            self, poly_x_q, poly_y_q, x_c, y_c):

        reach_aarect_int_grid_discretized = []
        for r in range(0, len(poly_x_q)):
            if not (poly_x_q[r].is_empty() or poly_y_q[r].is_empty()):
                x_min = poly_x_q[r].min_x()
                x_max = poly_x_q[r].max_x()
                y_min = poly_y_q[r].min_x()
                y_max = poly_y_q[r].max_x()

                ix_min = int(math.floor((x_min - x_c) / self.grid_x + 0.00001))
                ix_max = int(math.ceil((x_max - x_c) / self.grid_x - 0.00001))
                iy_min = int(math.floor((y_min - y_c) / self.grid_y + 0.00001))
                iy_max = int(math.ceil((y_max - y_c) / self.grid_y - 0.00001))

                n_rect_x = ix_max - ix_min
                n_rect_y = iy_max - iy_min

                for ix in range(0, n_rect_x):
                    for iy in range(0, n_rect_y):
                        reach_aarect_int_grid_discretized.append(
                            pycrreach.AARectInt(ix_min + ix,
                                ix_min + (ix + 1),
                                iy_min + iy,
                                iy_min + (iy + 1)))
        return reach_aarect_int_grid_discretized


    def find_bottom_left_top_right_of_reach_set_projection(self, poly_x_q, poly_y_q):
        # min
        x_c_lo = poly_x_q[0].min_x()
        x_c_hi = poly_x_q[0].max_x()
        for p in poly_x_q:
            x_c_lo = x_c_lo if x_c_lo <= p.min_x() else p.min_x()
            x_c_hi = x_c_hi if x_c_hi >= p.max_x() else p.max_x()

        # max
        y_c_lo = poly_y_q[0].min_x()
        y_c_hi = poly_y_q[0].max_x()
        for p in poly_y_q:
            y_c_lo = y_c_lo if y_c_lo <= p.min_x() else p.min_x()
            y_c_hi = y_c_hi if y_c_hi >= p.max_x() else p.max_x()

        return x_c_lo, y_c_lo, x_c_hi, y_c_hi


    def project_and_partition(self, poly_x_q: Polyhedron2VRep, poly_y_q: Polyhedron2VRep, collision_checker,
                              current_time):
        x_ref_lo, y_ref_lo = self.find_bottom_left_of_reach_set_projection(
            poly_x_q, poly_y_q)

        # complete gridded discretization
        reach_aarect_int_grid_discretized = (
            self.create_reach_aarect_grid_int_from_reach_set_ignore_empty(
                poly_x_q, poly_y_q, x_ref_lo, y_ref_lo))

        # # merge
        # reach_aarect_int_after_merge = pycrreach.merge_and_split_aarect_int(
        #     reach_aarect_int_before_merge)

        # undiscretize
        reach_aarect_double_after_merge = self.undiscretize(
            reach_aarect_int_grid_discretized, x_ref_lo, y_ref_lo)

        # # split
        # reach_aarect_double_after_split = pycrreach.split_aarect_double(
        #     collision_checker, reach_aarect_double_after_merge,
        #     current_time, self.max_radius)[0]

        # plt.gca().set_aspect('equal')
        # limits = [-5, 20, -5, 5]
        # draw_object(reach_aarect_int_grid_discretized)
        # plt.tight_layout()
        # plt.xlim([limits[0], limits[1]])
        # plt.ylim([limits[2], limits[3]]of)
        # plt.show()
        return reach_aarect_double_after_merge

    def propagate_multiple_steps(self, reachboxes, delta_t: float, reach_n_u_x, reach_n_u_y):
        """
        Propagates reachboxes over multiple timesteps
        :param reachboxes:
        :param delta_t: total time of propagation, delta_t= dt*n_timesteps
        :param reach_n_u_x:
        :param reach_n_u_x:
        :return:
        """
        def propagate_poly(poly, reach_u):
            x = poly.copy()
            x.linear_map(1, delta_t, 0, 1)
            x = x.minkowski_sum(reach_u)
            x.intersect_halfspace(0, 1, self.max_speed_x)
            x.intersect_halfspace(0, -1, -self.min_speed_x)
            return x

        poly_x_q = [propagate_poly(p.reach_x, reach_n_u_x) for p in reachboxes]
        poly_y_q = [propagate_poly(p.reach_y, reach_n_u_y) for p in reachboxes]
        return poly_x_q, poly_y_q

    def forward_multiple_steps(self, time_slice_current, time_slice_forward, reach_set_rects, delta_timesteps: int, reach_n_u_x, reach_n_u_y):
        """

        :param time_slice_current:
        :param time_slice_forward:
        :param reach_set_rects:
        :param delta_nt:
        :param reach_n_u_x:
        :param reach_n_u_y:
        :return:
        """
        poly_x_n, poly_y_n = self.propagate_multiple_steps(time_slice_current, delta_timesteps * self.dt, reach_n_u_x,
                                                           reach_n_u_y)

        # find reachable rects for every poly at t+delta_nt
        adj = self.determine_position_overlap(poly_x_n, poly_y_n, reach_set_rects)
        adj.sort(key=lambda x: x[1])  # (parent_id, child_id)

        # # convert to ReachSetNodeMultiGeneration
        # if type(time_slice_current[0]) is not ReachSetNodeMultiGeneration:
        #     new_timeslice_current = list()
        #     for reachset in time_slice_current:
        #         new_timeslice_current.append(ReachSetNodeMultiGeneration.from_ReachSetNode(reachset))
        #     time_slice_current = new_timeslice_current
        # if type(time_slice_forward[0]) is not ReachSetNodeMultiGeneration:
        #     new_timeslice_forward = list()
        #     for reachset in time_slice_forward:
        #         new_timeslice_forward.append(ReachSetNodeMultiGeneration.from_ReachSetNode(reachset))
        #     time_slice_forward = new_timeslice_forward

        # link Nodes
        for relation in adj:
            grandchild_id = relation[1]
            parent_id = relation[0]
            time_slice_current[parent_id].add_grandchild(delta_timesteps-1, grandchild_id)
            time_slice_forward[grandchild_id].add_grandparent(delta_timesteps-1, parent_id)


