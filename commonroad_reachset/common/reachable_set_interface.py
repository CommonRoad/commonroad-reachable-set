from collections import defaultdict
from enum import Enum, unique
from typing import Dict, List, Union

# import pycrreachset as reach

from commonroad_reachset.common.collision_checker import CollisionChecker
from commonroad_reachset.common.data_structure.configuration import Configuration
from commonroad_reachset.common.data_structure.reach_node import ReachNode
from commonroad_reachset.common.data_structure.reach_polygon import ReachPolygon
from commonroad_reachset.continuous.reachability_analysis import ReachabilityAnalysis


class CPPReachableSetInterface:
    """Interface to work with reachable sets."""

    @classmethod
    def continuous(cls, config: Configuration):
        config_cpp = config.convert_to_cpp_configuration()
        collision_checker = CollisionChecker(config)

        reachability_analysis = reach.ReachabilityAnalysis(config_cpp,
                                                           collision_checker.collision_checker_cpp)

        return CPPReachableSetInterface(reachability_analysis)

    def __init__(self, reachability_analysis: reach.ReachabilityAnalysis):
        self._reachability_analysis = reachability_analysis

        self._time_step_initial = reachability_analysis.config().planning.time_step_start
        self._dict_time_to_drivable_area = dict()
        self._dict_time_to_list_base_sets_propagated = dict()
        self._dict_time_to_reachable_set = dict()

        self._initialize()

    @property
    def config(self) -> Configuration:
        return self._reachability_analysis.config()

    @property
    def dict_time_to_drivable_area(self) -> Dict[int, List[ReachPolygon]]:
        """Dictionary holding drivable area at different time steps."""
        return self._dict_time_to_drivable_area

    @property
    def dict_time_to_reachable_set(self) -> Dict[int, List[ReachNode]]:
        """Dictionary holding reachable set at different time steps."""
        return self._dict_time_to_reachable_set

    def _initialize(self):
        """Initializes drivable area and reachable set of initial time step."""
        self._dict_time_to_drivable_area[
            self._time_step_initial
        ] = self._reachability_analysis.initial_drivable_area()

        self._dict_time_to_reachable_set[
            self._time_step_initial
        ] = self._reachability_analysis.initial_reachable_set()

    def compute(self, time_step_start: int = 1, time_step_end: int = 0):
        """Compute reachable set between the specified time steps."""
        if time_step_start == 0:
            print("Time step should not start with 0.")
            return

        if not time_step_end:
            time_step_end = self.config.planning.time_steps_computation

        for time_step in range(time_step_start, time_step_end + 1):
            self._compute_at_time_step(time_step)

    def _compute_at_time_step(self, time_step: int):
        """Compute drivable area and reachable set of the given time step."""
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Compute drivable area of the time step.

        Drivable area is computed based on reachable set of the last time step.
        """
        # prepare
        reachable_set_time_step_last = self._dict_time_to_reachable_set[time_step - 1]
        # compute
        drivable_area, list_base_sets_propagated = \
            self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_time_step_last)

        # save
        self._dict_time_to_drivable_area[time_step] = drivable_area
        self._dict_time_to_list_base_sets_propagated[time_step] = list_base_sets_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Compute reachable set of the time step.

        Reachable set is computed based on the drivable area and the list of
        propagated nodes. It is also based on the reachable set of the last time
        step.
        """
        # prepare
        reachable_set_time_step_previous = self._dict_time_to_reachable_set[time_step - 1]
        list_base_sets_propagated = self._dict_time_to_list_base_sets_propagated[time_step]
        drivable_area = self._dict_time_to_drivable_area[time_step]
        # compute
        reachable_set = self._reachability_analysis.compute_reachable_set_at_time_step(
            time_step,
            reachable_set_time_step_previous,
            list_base_sets_propagated,
            drivable_area,
        )
        # save
        self._dict_time_to_reachable_set[time_step] = reachable_set


class PyReachableSetInterface:
    """Interface to work with reachable sets."""

    @unique
    class Mode(Enum):
        CONTINUOUS = "continuous"
        SEMANTIC = "semantic"

    def __init__(self, reachability_analysis: ReachabilityAnalysis):
        self._reachability_analysis = reachability_analysis
        if isinstance(reachability_analysis, ReachabilityAnalysis):
            self._mode = PyReachableSetInterface.Mode.CONTINUOUS

        else:
            raise Exception("<ReachableSetInterface> Input reachability analysis is not valid.")

        self.time_step_start = reachability_analysis.config.planning.time_step_start
        self.time_step_end = reachability_analysis.config.planning.time_steps_computation + self.time_step_start

        self.dict_time_to_base_sets_propagated = dict()
        self.dict_time_to_drivable_area = dict()
        self.dict_time_to_reachable_set = dict()

        self.dict_time_to_drivable_area_refined = defaultdict(list)
        self.dict_time_to_reachable_set_refined = defaultdict(list)

        self._initialize()

        # for semantic mode
        self.transition_system_built = False
        self.dict_time_to_propositions_to_kripke_node = defaultdict(dict)
        self.dict_time_to_kripke_nodes = defaultdict(list)
        self.dict_time_to_list_nodes_kripke_accepted = defaultdict(list)
        self.reaches_final_time_step_after_ctl = True

    def _initialize(self):
        """Initializes drivable area and reachable set of initial time step."""
        self.dict_time_to_drivable_area[self.time_step_start] = self._reachability_analysis.initial_drivable_area

        self.dict_time_to_reachable_set[self.time_step_start] = self._reachability_analysis.initial_reachable_set

    @property
    def config(self) -> Configuration:
        return self._reachability_analysis.config

    @property
    def reachability_analysis(self):
        return self._reachability_analysis

    @property
    def semantic_model(self):
        if self._mode != PyReachableSetInterface.Mode.SEMANTIC:
            raise Exception("<PyReachableSetInterface> Only semantic reachability analysis has semantic model")

        return self._reachability_analysis.semantic_model

    @classmethod
    def continuous(cls, config: Configuration):
        reachability_analysis = ReachabilityAnalysis(config)

        return PyReachableSetInterface(reachability_analysis)

    @classmethod
    def semantic(cls, config: Configuration, semantic_model: SemanticModel):
        reachability_analysis = SemanticReachabilityAnalysis(config, semantic_model)

        return PyReachableSetInterface(reachability_analysis)

    def drivable_area_at_time_step(self, time_step: int) -> List[ReachPolygon]:
        assert time_step in self.dict_time_to_drivable_area, "<ReachableSetInterface> Input time step is not valid."

        list_drivable_area = []
        if self._mode == PyReachableSetInterface.Mode.CONTINUOUS:
            list_drivable_area = self.dict_time_to_drivable_area[time_step]

        elif self._mode == PyReachableSetInterface.Mode.SEMANTIC:
            for list_rectangles_drivable_area in self.dict_time_to_drivable_area[time_step].values():
                list_drivable_area.extend(list_rectangles_drivable_area)

        return list_drivable_area

    def reachable_set_at_time_step(self, time_step: int) -> List[ReachNode]:
        assert time_step in self.dict_time_to_drivable_area, "<ReachableSetInterface> Input time step is not valid."

        list_reachable_set = []
        if self._mode == PyReachableSetInterface.Mode.CONTINUOUS:
            list_reachable_set = self.dict_time_to_reachable_set[time_step]

        elif self._mode == PyReachableSetInterface.Mode.SEMANTIC:
            for list_nodes_reachable_set in self.dict_time_to_reachable_set[time_step].values():
                list_reachable_set.extend(list_nodes_reachable_set)

        return list_reachable_set

    def refined_reachable_set_at_time_step(self, time_step: int) -> List[ReachNode]:
        assert time_step in self.dict_time_to_drivable_area, "<ReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_reachable_set_refined[time_step]

    def compute(self, time_step_start: int = 1, time_step_end: int = 0):
        """Compute reachable set between the specified time steps.

        If the graph is to be pruned, those nodes not reaching the final step is discarded.
        """
        if time_step_start == 0:
            print("Time step should not start with 0.")
            return

        if not time_step_end:
            time_step_end = self.time_step_end

        for time_step in range(time_step_start, time_step_end + 1):
            self._compute_at_time_step(time_step)

        if self.config.reachable_set.prune_sets_not_reaching_last_time_step:
            self._prune_reachable_set()

    def _compute_at_time_step(self, time_step: int):
        """Computes drivable area and reachable set of the given time step."""
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Computes drivable area of the time step.

        Drivable area is computed based on reachable set of the last time step.
        """
        reachable_set_previous = self.dict_time_to_reachable_set[time_step - 1]

        if self._mode == PyReachableSetInterface.Mode.CONTINUOUS:
            drivable_area, base_sets_propagated = \
                self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_previous)

            self.dict_time_to_drivable_area[time_step] = drivable_area
            self.dict_time_to_base_sets_propagated[time_step] = base_sets_propagated

        elif self._mode == PyReachableSetInterface.Mode.SEMANTIC:
            dict_propositions_to_drivable_area, dict_propositions_to_list_base_sets_adapted = \
                self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_previous)

            self.dict_time_to_drivable_area[time_step] = dict_propositions_to_drivable_area
            self.dict_time_to_base_sets_propagated[time_step] = dict_propositions_to_list_base_sets_adapted

    def _compute_reachable_set_at_time_step(self, time_step):
        """Compute reachable set of the time step.

        Reachable set is computed based on the drivable area and the list of
        propagated nodes. It is also based on the reachable set of the last time
        step.
        """
        if self._mode == PyReachableSetInterface.Mode.CONTINUOUS:
            base_sets_propagated = self.dict_time_to_base_sets_propagated[time_step]
            drivable_area = self.dict_time_to_drivable_area[time_step]

            reachable_set = self._reachability_analysis. \
                compute_reachable_set_at_time_step(time_step, base_sets_propagated, drivable_area)

            self.dict_time_to_reachable_set[time_step] = reachable_set

        elif self._mode == PyReachableSetInterface.Mode.SEMANTIC:
            dict_propositions_to_list_base_sets_adapted = self.dict_time_to_base_sets_propagated[time_step]
            dict_propositions_to_drivable_area = self.dict_time_to_drivable_area[time_step]

            dict_propositions_to_reachable_sets = self._reachability_analysis.compute_reachable_set_at_time_step(
                time_step, dict_propositions_to_list_base_sets_adapted, dict_propositions_to_drivable_area)

            self.dict_time_to_reachable_set[time_step] = dict_propositions_to_reachable_sets

    def _prune_reachable_set(self):
        """Iterates through reachable sets in backward direction and discard nodes not reaching final time step."""
        cnt_nodes_before_prune = 0
        cnt_nodes_after_prune = 0

        for time_step in range(self.time_step_end - 1, self.time_step_start - 1, -1):
            list_nodes = self.reachable_set_at_time_step(time_step)
            dict_propositions_to_reachable_sets = self.dict_time_to_reachable_set[time_step]
            cnt_nodes_before_prune += len(list_nodes)

            for node in list_nodes:
                # discard if the node has no child node
                if not node.list_nodes_child:
                    # iterate through parent nodes and disconnect them
                    for node_parent in node.list_nodes_parent:
                        node_parent.remove_child_node(node)
                    node.list_nodes_parent.clear()

                    # remove the node from dict_time_to_reachable_set
                    for list_reachable_sets in dict_propositions_to_reachable_sets.values():
                        if node in list_reachable_sets:
                            list_reachable_sets.remove(node)

            for list_reachable_sets in dict_propositions_to_reachable_sets.values():
                cnt_nodes_after_prune += len(list_reachable_sets)

        print(f"\t#nodes before pruning: {cnt_nodes_before_prune}")
        print(f"\t#nodes after pruning: {cnt_nodes_after_prune}")

    def construct_transition_system(self):
        """Constructs transition system from the reachability graph.

        At every time step, a kripke node may contain one or multiple reachability nodes with the same set of
        propositions. The constructed transition system is later used for model checking and maneuver extraction.
        A dictionary is constructed to map time step to sets of propositions to kripke node.
        """
        if self._mode != PyReachableSetInterface.Mode.SEMANTIC:
            raise Exception("<ReachableSetInterface> Transition system can only be constructed in semantic mode.")

        for time_step in range(self.time_step_end + 1):
            dict_propositions_to_kripke_node = self.dict_time_to_propositions_to_kripke_node[time_step]

            for node_reach in self.reachable_set_at_time_step(time_step):
                node_kripke = None
                try:
                    # retrieve existing kripke node for the reachability node
                    node_kripke = dict_propositions_to_kripke_node[node_reach.set_propositions_persistent]
                    node_kripke.add_reach_node(node_reach)

                except KeyError:
                    # if such kripke node does not exist, create a new one
                    node_kripke = KripkeNode(time_step, [node_reach], node_reach.set_propositions_persistent)
                    dict_propositions_to_kripke_node[node_reach.set_propositions_persistent] = node_kripke

                finally:
                    # add connections between the kripke nodes based on the parent-child relationship of the
                    # reachability nodes
                    for node_reach_parent in node_reach.list_nodes_parent:
                        node_kripke_parent = self.dict_time_to_propositions_to_kripke_node[time_step - 1][
                            node_reach_parent.set_propositions_persistent]

                        node_kripke_parent.add_child_node(node_kripke)
                        node_kripke.add_parent_node(node_kripke_parent)

            # create a dictionary that maps time steps to
            self.dict_time_to_kripke_nodes[time_step] += dict_propositions_to_kripke_node.values()

        self.transition_system_built = True

    def print_info(self):
        print(f"\tTime steps: {self.config.planning.time_steps_computation}, "
              f"use SPOT: {self.config.planning.use_spot}, "
              f"prune: {self.config.reachable_set.prune_sets_not_reaching_last_time_step}")
