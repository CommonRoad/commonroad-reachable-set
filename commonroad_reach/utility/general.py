from itertools import chain, combinations

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet


def load_scenario_and_planning_problem(config, idx_planning_problem: int = 0):
    scenario, planning_problem_set = CommonRoadFileReader(config.general.path_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[idx_planning_problem]

    return scenario, planning_problem


def power_set(iterable):
    """Returns the power set of a given iterable"""
    return set(chain.from_iterable(combinations(iterable, r) for r in range(1, len(iterable) + 1)))


def create_lanelet_network_from_ids(lanelet_network: LaneletNetwork, lanelets_leading_to_goal) -> LaneletNetwork:
    """
    creates a new lanelet network from a list of lanelet IDs in the original lanelet network
    """
    new_lanelet_network = LaneletNetwork()

    for lanelet_id in lanelets_leading_to_goal:
        lanelet_orig = lanelet_network.find_lanelet_by_id(lanelet_id)

        predecessor = list(set(lanelet_orig.predecessor).intersection(lanelets_leading_to_goal))
        successor = list(set(lanelet_orig.successor).intersection(lanelets_leading_to_goal))

        lanelet = Lanelet(lanelet_orig.left_vertices, lanelet_orig.center_vertices, lanelet_orig.right_vertices,
                          lanelet_orig.lanelet_id, predecessor, successor)

        if {lanelet_orig.adj_left}.intersection(lanelets_leading_to_goal):
            lanelet.adj_left = lanelet_orig.adj_left
            lanelet.adj_left_same_direction = lanelet_orig.adj_left_same_direction
        if {lanelet_orig.adj_right}.intersection(lanelets_leading_to_goal):
            lanelet.adj_right = lanelet_orig.adj_right
            lanelet.adj_right_same_direction = lanelet_orig.adj_right_same_direction
        new_lanelet_network.add_lanelet(lanelet)
    return new_lanelet_network