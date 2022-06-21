from typing import Dict, List, Union

from commonroad_reach import pycrreach
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.utility import geometry as util_geometry

# scaling factor (avoid numerical errors)
DIGITS = 2


class ConnectedComponent:
    cnt_id = 0

    def __init__(self, list_nodes_reach: List[Union[pycrreach.ReachNode, ReachNode]] = None):
        self.list_nodes_reach = list_nodes_reach
        self.area = util_geometry.area_of_reachable_set(list_nodes_reach)
        self.step = list_nodes_reach[0].step

        self.id = ConnectedComponent.cnt_id
        ConnectedComponent.cnt_id += 1

    def __repr__(self):
        return f"ConnectedComponent(step={self.step}, #nodes={len(self.list_nodes_reach)}, area={self.area})"


class DrivingCorridor:
    def __init__(self):
        self.dict_step_to_cc: Dict[int, ConnectedComponent] = dict()
        self.area = 0

    @property
    def step_final(self):
        return max(self.dict_step_to_cc)

    def __repr__(self):
        return f"DrivingCorridor(step_final={self.step_final}, #CC={len(self.dict_step_to_cc)}, area={self.area})"

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
