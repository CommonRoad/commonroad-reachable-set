import copy
from collections import defaultdict
from typing import Optional, Dict, Set, List

from shapely import affinity

from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


class ReachNode:
    """
    Node within a reachability graph, also used in reachable set computations.

    .. note::
        - Each node is a Cartesian product of longitudinal and lateral polygons.
        - Curvilinear coordinate system: polygon_lon is a polygon in the longitudinal p-v domain,
          and polygon_lat is a polygon in the lateral p-v domain.
        - Cartesian coordinate system: polygons are in the x-v and y-v domains, respectively.
    """
    cnt_id = 0

    def __init__(self, polygon_lon: ReachPolygon, polygon_lat: ReachPolygon, step: int = -1):
        self._polygon_lon: ReachPolygon = polygon_lon
        self._polygon_lat: ReachPolygon = polygon_lat
        self._bounds_lon = polygon_lon.bounds if polygon_lon else None
        self._bounds_lat = polygon_lat.bounds if polygon_lat else None

        self.position_rectangle: Optional[ReachPolygon] = None
        if self._bounds_lon and self._bounds_lat:
            self.update_position_rectangle()

        self.id = ReachNode.cnt_id
        ReachNode.cnt_id += 1
        self.step = step
        self.list_nodes_parent: List[ReachNode] = list()
        self.list_nodes_child: List[ReachNode] = list()

        # the node from which the current node is propagated
        self.source_propagation = None

    def __repr__(self):
        return f"ReachNode(step={self.step}, id={self.id})"

    def __eq__(self, other: object) -> bool:
        if isinstance(other, ReachNode):
            return self.id == other.id and self.step == other.step

        else:
            return False

    def __key(self):
        return self.id, self.step

    def __hash__(self):
        return hash(self.__key())

    @property
    def polygon_lon(self) -> ReachPolygon:
        """
        Polygon in the longitudinal direction. See note of :class:`ReachNode`.
        """
        return self._polygon_lon

    @polygon_lon.setter
    def polygon_lon(self, polygon: ReachPolygon):
        self._polygon_lon = polygon
        self._bounds_lon = polygon.bounds

    @property
    def polygon_lat(self) -> ReachPolygon:
        """
        Polygon in the lateral direction. See note of :class:`ReachNode`.
        """
        return self._polygon_lat

    @polygon_lat.setter
    def polygon_lat(self, polygon: ReachPolygon):
        self._polygon_lat = polygon
        self._bounds_lat = polygon.bounds

    @property
    def p_lon_min(self):
        """
        Minimum position in the longitudinal direction.
        """
        return self._bounds_lon[0]

    @property
    def p_lon_max(self):
        """
        Maximum position in the longitudinal direction.
        """
        return self._bounds_lon[2]

    @property
    def v_lon_min(self):
        """
        Minimum velocity in the longitudinal direction.
        """
        return self._bounds_lon[1]

    @property
    def v_lon_max(self):
        """
        Maximum velocity in the longitudinal direction.
        """
        return self._bounds_lon[3]

    @property
    def p_lat_min(self):
        """
        Minimum position in the lateral direction.
        """
        return self._bounds_lat[0]

    @property
    def p_lat_max(self):
        """
        Maximum position in the lateral direction.
        """
        return self._bounds_lat[2]

    @property
    def v_lat_min(self):
        """
        Minimum velocity in the lateral direction.
        """
        return self._bounds_lat[1]

    @property
    def v_lat_max(self):
        """
        Maximum velocity in the lateral direction.
        """
        return self._bounds_lat[3]

    @property
    def p_x_min(self):
        """
        Minimum x-position in the Cartesian coordinate system.
        """
        return self.p_lon_min

    @property
    def p_x_max(self):
        """
        Maximum x-position in the Cartesian coordinate system.
        """
        return self.p_lon_max

    @property
    def v_x_min(self):
        """
        Minimum x-velocity in the Cartesian coordinate system.
        """
        return self.v_lon_min

    @property
    def v_x_max(self):
        """
        Maximum x-velocity in the Cartesian coordinate system.
        """
        return self.v_lon_max

    @property
    def p_y_min(self):
        """
        Minimum y-position in the Cartesian coordinate system.
        """
        return self.p_lat_min

    @property
    def p_y_max(self):
        """
        Maximum y-position in the Cartesian coordinate system.
        """
        return self.p_lat_max

    @property
    def v_y_min(self):
        """
        Minimum y-velocity in the Cartesian coordinate system.
        """
        return self.v_lat_min

    @property
    def v_y_max(self):
        """
        Maximum y-velocity in the Cartesian coordinate system.
        """
        return self.v_lat_max

    def clone(self) -> "ReachNode":
        """
        Returns a clone of the reach node.
        """
        node_clone = ReachNode(self.polygon_lon.clone(convexify=False),
                               self.polygon_lat.clone(convexify=False),
                               self.step)
        node_clone.list_nodes_parent = copy.deepcopy(self.list_nodes_parent)
        node_clone.list_nodes_child = copy.deepcopy(self.list_nodes_child)
        node_clone.source_propagation = self.source_propagation

        return node_clone

    def update_position_rectangle(self):
        """
        Updates the position rectangle based on the latest position attributes.
        """
        tuple_vertices_rectangle = (self.p_lon_min, self.p_lat_min, self.p_lon_max, self.p_lat_max)

        self.position_rectangle = ReachPolygon.from_rectangle_vertices(*tuple_vertices_rectangle)

    def translate(self, p_lon_off: float = 0.0, v_lon_off: float = 0.0,
                  p_lat_off: float = 0.0, v_lat_off: float = 0.0):
        """
        Returns a copy translated by input offsets.
        """
        return ReachNode(
            ReachPolygon.from_polygon(
                affinity.translate(self.polygon_lon.shapely_object, xoff=p_lon_off, yoff=v_lon_off)),
            ReachPolygon.from_polygon(
                affinity.translate(self.polygon_lat.shapely_object, xoff=p_lat_off, yoff=v_lat_off)),
            step=self.step)

    def add_parent_node(self, node_parent: "ReachNode"):
        if node_parent not in self.list_nodes_parent:
            self.list_nodes_parent.append(node_parent)

    def remove_parent_node(self, node_parent: "ReachNode") -> bool:
        if node_parent in self.list_nodes_parent:
            self.list_nodes_parent.remove(node_parent)
            return True

        return False

    def add_child_node(self, node_child: "ReachNode"):
        if node_child not in self.list_nodes_child:
            self.list_nodes_child.append(node_child)

    def remove_child_node(self, node_child: "ReachNode") -> bool:
        if node_child in self.list_nodes_child:
            self.list_nodes_child.remove(node_child)
            return True

        return False

    def intersect_in_position_domain(self, p_lon_min: Optional[float] = None, p_lat_min: Optional[float] = None,
                                     p_lon_max: Optional[float] = None, p_lat_max: Optional[float] = None):
        """
        Perform intersection in the position domain.
        """
        if self._is_halfspace_intersection_necessary(self.p_lon_max, p_lon_max, is_max=True):
            self._polygon_lon = self.polygon_lon.intersect_halfspace(1, 0, p_lon_max)
        if self._is_halfspace_intersection_necessary(self.p_lon_min, p_lon_min, is_max=False):
            self._polygon_lon = self.polygon_lon.intersect_halfspace(-1, 0, -p_lon_min)
        if self._is_halfspace_intersection_necessary(self.p_lat_max, p_lat_max, is_max=True):
            self._polygon_lat = self.polygon_lat.intersect_halfspace(1, 0, p_lat_max)
        if self._is_halfspace_intersection_necessary(self.p_lat_min, p_lat_min, is_max=False):
            self._polygon_lat = self.polygon_lat.intersect_halfspace(-1, 0, -p_lat_min)

        if not self.is_empty:
            self._bounds_lon = self._polygon_lon.bounds
            self._bounds_lat = self._polygon_lat.bounds
            self.update_position_rectangle()

    def intersect_in_velocity_domain(self, v_lon_min: Optional[float] = None, v_lat_min: Optional[float] = None,
                                     v_lon_max: Optional[float] = None, v_lat_max: Optional[float] = None):
        """
        Perform intersection in the velocity domain.
        """
        if self._is_halfspace_intersection_necessary(self.v_lon_max, v_lon_max, is_max=True):
            self._polygon_lon = self.polygon_lon.intersect_halfspace(0, 1, v_lon_max)
        if self._is_halfspace_intersection_necessary(self.v_lon_min, v_lon_min, is_max=False):
            self._polygon_lon = self.polygon_lon.intersect_halfspace(0, -1, -v_lon_min)
        if self._is_halfspace_intersection_necessary(self.v_lat_max, v_lat_max, is_max=True):
            self._polygon_lat = self.polygon_lat.intersect_halfspace(0, 1, v_lat_max)
        if self._is_halfspace_intersection_necessary(self.v_lat_min, v_lat_min, is_max=False):
            self._polygon_lat = self.polygon_lat.intersect_halfspace(0, -1, -v_lat_min)

    def _is_halfspace_intersection_necessary(self, current: float, target: Optional[float], *,
                                             is_max: bool) -> bool:
        """
        Check whether it is necessary to perform a halfspace intersection given current and target bounds.

        :param current: current bound of the polygon
        :param target: target bound of the polygon
        :param is_max: if True, current and target bound are treated as maximal bounds, otherwise as minimal bounds.
        :returns: True iff the halfspace intersection is necessary.
        """
        if is_max:
            return not self.is_empty and target is not None and current > target
        else:
            return not self.is_empty and target is not None and current < target

    @property
    def is_empty(self) -> bool:
        """
        Returns True iff the reach node is empty.
        """
        return self.polygon_lon is None or self.polygon_lat is None or self.polygon_lon.is_empty or self.polygon_lat.is_empty

    @classmethod
    def reset_class_id_counter(cls):
        cls.cnt_id = 0


class ReachNodeMultiGeneration(ReachNode):
    """
    Node within a reachability graph, also used in reachable set computations.

    In addition to :class:`ReachNode`, this class holds lists reach nodes across generations.
    """

    def __init__(self, polygon_lon, polygon_lat, step: int = -1):
        super(ReachNodeMultiGeneration, self).__init__(polygon_lon, polygon_lat, step)
        self.dict_time_to_set_nodes_grandparent: Dict[int, Set[ReachNodeMultiGeneration]] = defaultdict(set)
        self.dict_time_to_set_nodes_grandchild: Dict[int, Set[ReachNodeMultiGeneration]] = defaultdict(set)

    def add_grandparent_node(self, node_grandparent: "ReachNodeMultiGeneration") -> bool:
        delta_steps = self.step - node_grandparent.step
        assert delta_steps > 1, f"not a grand_parent: node_grandparent.step={node_grandparent.step}, " \
                                f"self.step={self.step}"
        if node_grandparent not in self.dict_time_to_set_nodes_grandparent[delta_steps]:
            self.dict_time_to_set_nodes_grandparent[delta_steps].add(node_grandparent)
            return True

        return False

    def remove_grandparent_node(self, node_grandparent: "ReachNodeMultiGeneration") -> bool:
        delta_steps = self.step - node_grandparent.step
        if node_grandparent in self.dict_time_to_set_nodes_grandparent[delta_steps]:
            self.dict_time_to_set_nodes_grandparent[delta_steps].remove(node_grandparent)
            return True

        return False

    def add_grandchild_node(self, node_grandchild: "ReachNodeMultiGeneration") -> bool:
        delta_steps = node_grandchild.step - self.step
        assert delta_steps > 1, f"not a grandchild: node_grandchild.step={node_grandchild.step}, " \
                                f"self.step={self.step}"
        if node_grandchild not in self.dict_time_to_set_nodes_grandchild[delta_steps]:
            self.dict_time_to_set_nodes_grandchild[delta_steps].add(node_grandchild)
            return True

        return False

    def remove_grandchild_node(self, node_grandchild: "ReachNodeMultiGeneration") -> bool:
        delta_steps = node_grandchild.step - self.step
        if node_grandchild in self.dict_time_to_set_nodes_grandchild[delta_steps]:
            self.dict_time_to_set_nodes_grandchild[delta_steps].remove(node_grandchild)
            return True

        return False
