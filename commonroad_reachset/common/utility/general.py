from itertools import chain, combinations

from commonroad.common.file_reader import CommonRoadFileReader


def load_scenario_and_planning_problem(config, idx_planning_problem: int = 0):
    scenario, planning_problem_set = CommonRoadFileReader(config.general.path_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[
        idx_planning_problem
    ]

    return scenario, planning_problem


def power_set(iterable):
    """Returns the power set of a given iterable"""
    return set(chain.from_iterable(combinations(iterable, r) for r in range(1, len(iterable) + 1)))
