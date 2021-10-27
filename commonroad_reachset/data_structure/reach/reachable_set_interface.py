from typing import List, Union
from collections import defaultdict

# import pycrreachset as reach
# from commonroad_reachset.common.collision_checker import CollisionChecker
import time

from commonroad_reachset.data_structure.configuration import Configuration
from commonroad_reachset.data_structure.reach.reach_node import ReachNode
from commonroad_reachset.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reachset.data_structure.reach.reachability_analysis import ReachabilityAnalysis


class ReachableSetInterface:
    """Interface to work with reachable sets."""

    def __init__(self, config: Configuration):
        self.config = config

        if config.reachable_set.back_end == "PYTHON":
            self.back_end = "PYTHON"
            self._interface = PyReachableSetInterface(config)

        elif config.reachable_set.back_end == "CPP":
            self.back_end = "CPP"

        else:
            raise Exception("<ReachableSetInterface> Specified backend is not valid.")

    @property
    def reachability_analysis(self):
        return self._interface.reachability_analysis

    @property
    def time_step_start(self):
        return self._interface.time_step_start

    @property
    def time_step_end(self):
        return self._interface.time_step_end

    def drivable_area_at_time_step(self, time_step: int) -> Union[List[ReachPolygon]]:
        return self._interface.drivable_area_at_time_step(time_step)

    def reachable_set_at_time_step(self, time_step: int) -> Union[List[ReachNode]]:
        return self._interface.reachable_set_at_time_step(time_step)

    def pruned_reachable_set_at_time_step(self, time_step: int) -> Union[List[ReachNode]]:
        return self._interface.pruned_reachable_set_at_time_step(time_step)

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        print("Computing reachable sets...")
        time_start = time.time()
        self._interface.compute_reachable_sets(time_step_start, time_step_end)

        if self.config.debug.measure_time:
            print(f"\tComputation took: {time.time() - time_start:.3f}s")


class PyReachableSetInterface:
    """Interface to work with reachable sets with python backend."""

    def __init__(self, config: Configuration):
        self.config = config
        self._reachability_analysis = ReachabilityAnalysis(config)
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self.dict_time_to_base_set_propagated = dict()
        self.dict_time_to_drivable_area = dict()
        self.dict_time_to_reachable_set = dict()
        self.dict_time_to_drivable_area_pruned = defaultdict(list)
        self.dict_time_to_reachable_set_pruned = defaultdict(list)

        self._initialize()

    def _initialize(self):
        """Initializes drivable area and reachable set of the initial time step."""
        self.dict_time_to_drivable_area[self.time_step_start] = self._reachability_analysis.initial_drivable_area
        self.dict_time_to_reachable_set[self.time_step_start] = self._reachability_analysis.initial_reachable_set

    @property
    def reachability_analysis(self):
        return self._reachability_analysis

    def drivable_area_at_time_step(self, time_step: int) -> List[ReachPolygon]:
        assert time_step in self.dict_time_to_drivable_area, "<ReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int) -> List[ReachNode]:
        assert time_step in self.dict_time_to_drivable_area, "<ReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_reachable_set[time_step]

    def pruned_reachable_set_at_time_step(self, time_step: int) -> List[ReachNode]:
        assert time_step in self.dict_time_to_drivable_area, "<ReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_reachable_set_pruned[time_step]

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        assert time_step_start != 0, "<ReachableSetInterface> Time step should not start with 0."

        if not time_step_end:
            time_step_end = self.time_step_end

        for time_step in range(time_step_start, time_step_end + 1):
            self._compute_at_time_step(time_step)

        if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
            self._prune_reachable_set()

    def _compute_at_time_step(self, time_step: int):
        """Computes drivable area and reachable set of the time step."""
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Computes drivable area of the time step.

        Drivable area is computed based on reachable set of the last time step.
        """
        reachable_set_previous = self.dict_time_to_reachable_set[time_step - 1]
        drivable_area, base_set_propagated = \
            self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_previous)

        self.dict_time_to_drivable_area[time_step] = drivable_area
        self.dict_time_to_base_set_propagated[time_step] = base_set_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Compute reachable set of the time step.

        Reachable set is computed based on the drivable area and the list of propagated nodes. It is also based on the
        reachable set of the last time step.
        """
        base_sets_propagated = self.dict_time_to_base_set_propagated[time_step]
        drivable_area = self.dict_time_to_drivable_area[time_step]

        reachable_set = self._reachability_analysis. \
            compute_reachable_set_at_time_step(time_step, base_sets_propagated, drivable_area)

        self.dict_time_to_reachable_set[time_step] = reachable_set

    def _prune_reachable_set(self):
        """Prunes nodes not reaching the final time step.

        Iterates through reachable sets in backward direction and discard nodes not reaching the final time step.
        """
        cnt_nodes_before_pruning = 0
        cnt_nodes_after_pruning = 0

        for time_step in range(self.time_step_end - 1, self.time_step_start - 1, -1):
            list_nodes = self.reachable_set_at_time_step(time_step)
            cnt_nodes_before_pruning += len(list_nodes)

            for node in list_nodes:
                # discard if the node has no child node
                if not node.list_nodes_child:
                    # iterate through parent nodes and disconnect them
                    for node_parent in node.list_nodes_parent:
                        node_parent.remove_child_node(node)
                    node.list_nodes_parent.clear()
                    list_nodes.remove(node)

            cnt_nodes_after_pruning += len(list_nodes)

        print(f"\t#nodes before pruning: {cnt_nodes_before_pruning}")
        print(f"\t#nodes after pruning: {cnt_nodes_after_pruning}")

# class CppReachableSetInterface:
#     """"""Interface to work with reachable sets (C++ backend).""""""
#
#     @classmethod
#     def continuous(cls, config: Configuration):
#         config_cpp = config.convert_to_cpp_configuration()
#         collision_checker = CollisionChecker(config)
#
#         reachability_analysis = reach.ReachabilityAnalysis(config_cpp,
#                                                            collision_checker.collision_checker_cpp)
#
#         return CPPReachableSetInterface(reachability_analysis)
#
#     def __init__(self, reachability_analysis: reach.ReachabilityAnalysis):
#         self._reachability_analysis = reachability_analysis
#
#         self._time_step_initial = reachability_analysis.config().planning.time_step_start
#         self._dict_time_to_drivable_area = dict()
#         self._dict_time_to_list_base_sets_propagated = dict()
#         self._dict_time_to_reachable_set = dict()
#
#         self._initialize()
#
#     @property
#     def config(self) -> Configuration:
#         return self._reachability_analysis.config()
#
#     @property
#     def dict_time_to_drivable_area(self) -> Dict[int, List[ReachPolygon]]:
#         """Dictionary holding drivable area at different time steps."""
#         return self._dict_time_to_drivable_area
#
#     @property
#     def dict_time_to_reachable_set(self) -> Dict[int, List[ReachNode]]:
#         """Dictionary holding reachable set at different time steps."""
#         return self._dict_time_to_reachable_set
#
#     def _initialize(self):
#         """Initializes drivable area and reachable set of initial time step."""
#         self._dict_time_to_drivable_area[
#             self._time_step_initial
#         ] = self._reachability_analysis.initial_drivable_area()
#
#         self._dict_time_to_reachable_set[
#             self._time_step_initial
#         ] = self._reachability_analysis.initial_reachable_set()
#
#     def compute(self, time_step_start: int = 1, time_step_end: int = 0):
#         """Compute reachable set between the specified time steps."""
#         if time_step_start == 0:
#             print("Time step should not start with 0.")
#             return
#
#         if not time_step_end:
#             time_step_end = self.config.planning.time_steps_computation
#
#         for time_step in range(time_step_start, time_step_end + 1):
#             self._compute_at_time_step(time_step)
#
#     def _compute_at_time_step(self, time_step: int):
#         """Compute drivable area and reachable set of the given time step."""
#         self._compute_drivable_area_at_time_step(time_step)
#         self._compute_reachable_set_at_time_step(time_step)
#
#     def _compute_drivable_area_at_time_step(self, time_step: int):
#         """Compute drivable area of the time step.
#
#         Drivable area is computed based on reachable set of the last time step.
#         """
#         # prepare
#         reachable_set_time_step_last = self._dict_time_to_reachable_set[time_step - 1]
#         # compute
#         drivable_area, list_base_sets_propagated = \
#             self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_time_step_last)
#
#         # save
#         self._dict_time_to_drivable_area[time_step] = drivable_area
#         self._dict_time_to_list_base_sets_propagated[time_step] = list_base_sets_propagated
#
#     def _compute_reachable_set_at_time_step(self, time_step):
#         """Compute reachable set of the time step.
#
#         Reachable set is computed based on the drivable area and the list of
#         propagated nodes. It is also based on the reachable set of the last time
#         step.
#         """
#         # prepare
#         reachable_set_time_step_previous = self._dict_time_to_reachable_set[time_step - 1]
#         list_base_sets_propagated = self._dict_time_to_list_base_sets_propagated[time_step]
#         drivable_area = self._dict_time_to_drivable_area[time_step]
#         # compute
#         reachable_set = self._reachability_analysis.compute_reachable_set_at_time_step(
#             time_step,
#             reachable_set_time_step_previous,
#             list_base_sets_propagated,
#             drivable_area,
#         )
#         # save
#         self._dict_time_to_reachable_set[time_step] = reachable_set
