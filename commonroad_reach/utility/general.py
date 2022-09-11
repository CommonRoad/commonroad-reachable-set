from typing import Tuple, Set
from itertools import chain, combinations

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.common.file_reader import CommonRoadFileReader


def load_scenario_and_planning_problem(config, idx_planning_problem: int = 0) -> Tuple[Scenario, PlanningProblem]:
    """
    Loads a scenario and planning problem from the configuration.

    :param config: configuration
    :param idx_planning_problem: index of the planning problem
    :return: scenario and planning problem
    """
    scenario, planning_problem_set = CommonRoadFileReader(config.general.path_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[idx_planning_problem]

    return scenario, planning_problem


def power_set(iterable) -> Set[Tuple]:
    """
    Returns the power set of a given iterable.
    """
    return set(chain.from_iterable(combinations(iterable, r) for r in range(1, len(iterable) + 1)))


def create_lanelet_network_from_ids(lanelet_network: LaneletNetwork, list_ids_lanelets) -> LaneletNetwork:
    """
    Creates a new lanelet network from a list of lanelet IDs in the original lanelet network.

    :param lanelet_network: reference lanelet network
    :param list_ids_lanelets: list of ids of lanelets from which the new lanelet network should be constructed
    :return: new lanelet network
    """
    new_lanelet_network = LaneletNetwork()

    for id_lanelet in list_ids_lanelets:
        lanelet_orig = lanelet_network.find_lanelet_by_id(id_lanelet)

        predecessor = list(set(lanelet_orig.predecessor).intersection(list_ids_lanelets))
        successor = list(set(lanelet_orig.successor).intersection(list_ids_lanelets))

        lanelet = Lanelet(lanelet_orig.left_vertices, lanelet_orig.center_vertices, lanelet_orig.right_vertices,
                          lanelet_orig.lanelet_id, predecessor, successor)

        if {lanelet_orig.adj_left}.intersection(list_ids_lanelets):
            lanelet.adj_left = lanelet_orig.adj_left
            lanelet.adj_left_same_direction = lanelet_orig.adj_left_same_direction

        if {lanelet_orig.adj_right}.intersection(list_ids_lanelets):
            lanelet.adj_right = lanelet_orig.adj_right
            lanelet.adj_right_same_direction = lanelet_orig.adj_right_same_direction

        new_lanelet_network.add_lanelet(lanelet)

    return new_lanelet_network
