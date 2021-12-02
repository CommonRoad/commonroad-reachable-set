from collections import defaultdict

from commonroad.geometry.shape import Polygon, Rectangle
import numpy as np
from typing import List, Union
# from commonroad_reach.reach.deprecated.cont_reach import ReachSetNode
# from reachbilityPruning.pruningPy.graph_reach.online_grid_reach import ReachSetNodeMultiGeneration
from pycrreach import ReachSetNode


def time_slice2shapes(time_slice):
    """
    Convert time_slice to shapes for plotting
    :param time_slice:
    :return:
    """

    def node2shaperect(node):
        xlo = float(node.x_min())
        xhi = node.x_max()
        ylo = node.y_min()
        yhi = node.y_max()
        verts = [[xlo, ylo], [xhi, ylo], [xhi, yhi], [xlo, yhi]]
        # verts = np.array([[xlo, xhi, xhi, xlo], [ylo, ylo, yhi, yhi]])
        return Polygon(verts)

    poly_list = list()
    for time_slice_i in time_slice:
        poly_list.append(list())
        for node in time_slice_i:
            poly_list[-1].append(node2shaperect(node))
    return poly_list


def xy2i_conversion(xy:List[int],ny:int) -> int:
    """
    converts x,y coordinates to index of a nx,ny matrix
    :param xy: [x,y]
    :param nx: shape[0] of matrix
    :return:
    """
    return xy[0] * ny + xy[1]

def i2xy(i,ny):
    return np.floor_divide(i,ny), np.mod(i,ny)

def compute_area_from_shapelist(shapes: Union[List[Union[Polygon, Rectangle]], List[List[Union[Polygon, Rectangle]]]], time_discrete=False):
    if time_discrete:
        # only works if shapes is nested list
        area = [0]
        for poly_list in shapes:
            tmp_area = 0
            for poly in poly_list:
                tmp_area += poly.shapely_object.area
            area.append(tmp_area)
        return np.array(area)
    else:
        area = 0
        for poly in shapes:
            area += poly.shapely_object.area
        return area


class ReachSetNodeMultiGeneration(ReachSetNode):
    def __init__(self, reach_x, reach_y, time_idx):
        ReachSetNode.__init__(self, reach_x, reach_y, time_idx)
        self.grandchild_nodes = defaultdict(list)  # {delta_timesteps: List[grandchild_ids: int]}
        self.grandparent_nodes = defaultdict(list)  # {delta_timesteps: List[grandparent_ids: int]}

    def add_grandchild(self, delta_timesteps: int, grandchild: 'ReachSetNodeMultiGeneration'):
        """
        Add grandchild delta_timestep generations forward
        :param delta_timesteps:
        :param grandchild:
        :return:
        """
        if not grandchild in self.grandchild_nodes[delta_timesteps]:
            self.grandchild_nodes[delta_timesteps].append(grandchild)

    def add_grandparent(self, delta_timesteps: int, grandparent: 'ReachSetNodeMultiGeneration'):
        """
        Add grandparent delta_timestep generations backward
        :param delta_timesteps:
        :param grandparent:
        :return:
        """
        if not grandparent in self.grandparent_nodes[delta_timesteps]:
            self.grandparent_nodes[delta_timesteps].append(grandparent)