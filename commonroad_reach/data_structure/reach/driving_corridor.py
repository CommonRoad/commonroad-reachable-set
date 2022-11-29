from typing import Dict, List, Union

from commonroad_reach import pycrreach
from commonroad_reach.data_structure.reach.reach_node import ReachNode
import commonroad_reach.utility.reach_operation


class ConnectedComponent:
    """
    Class representing a list of connected :class:`~commonroad_reach.data_structure.reach.reach_node.ReachNode`.
    """
    cnt_id = 0

    def __init__(self, list_nodes_reach: List[Union[pycrreach.ReachNode, ReachNode]] = None):
        self.list_nodes_reach = list_nodes_reach
        self.step = list_nodes_reach[0].step if list_nodes_reach else None
        self.area = commonroad_reach.utility.reach_operation.compute_area_of_reach_nodes(list_nodes_reach)

        self.id = ConnectedComponent.cnt_id
        ConnectedComponent.cnt_id += 1

    def __repr__(self):
        return f"ConnectedComponent(step={self.step}, #nodes={len(self.list_nodes_reach)}, area={self.area})"

    def __len__(self):
        return len(self.list_nodes_reach)

    def __getitem__(self, item):
        return self.list_nodes_reach[item]


class DrivingCorridor:
    """
    Class representing a sequence of :class:`ConnectedComponent`.
    """

    def __init__(self):
        self.dict_step_to_cc: Dict[int, ConnectedComponent] = dict()
        self.area = 0

    @property
    def step_final(self):
        return max(self.dict_step_to_cc)

    def __repr__(self):
        return f"DrivingCorridor(step_final={self.step_final}, #CC={len(self.dict_step_to_cc)}, area={self.area})"

    def __len__(self):
        return len(self.dict_step_to_cc)

    def __getitem__(self, name):
        return self.dict_step_to_cc[name]

    def __iter__(self):
        return iter(self.dict_step_to_cc)

    def keys(self):
        return self.dict_step_to_cc.keys()

    def items(self):
        return self.dict_step_to_cc.items()

    def values(self):
        return self.dict_step_to_cc.values()

    def add_connected_component(self, cc: ConnectedComponent):
        self.dict_step_to_cc[cc.step] = cc
        self.area += cc.area

    def connected_component_at_step(self, step: int):
        return self.dict_step_to_cc[step]

    def connected_components(self):
        return self.dict_step_to_cc

    def reach_nodes(self):
        dict_step_to_list_nodes_reach = dict()
        for step, cc in self.dict_step_to_cc.items():
            dict_step_to_list_nodes_reach[step] = cc.list_nodes_reach

        return dict_step_to_list_nodes_reach

    def reach_nodes_at_step(self, step: int):
        return self.connected_component_at_step(step).list_nodes_reach
