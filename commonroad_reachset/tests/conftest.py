"""Shared fixtures"""
import os
import sys

import pytest

from commonroad_reachset.common.collision_checker import CollisionChecker
from commonroad_reachset.common.configuration_builder import ConfigurationBuilder
from commonroad_reachset.continuous.continuous_reachability_analysis import \
    ContinuousReachabilityAnalysis
from commonroad_reachset.common.data_structure.reach_node import ReachNode
from commonroad_reachset.common.data_structure.reach_polygon import ReachPolygon

sys.path.append(os.getcwd())


@pytest.fixture
def path_config():
    return "/home/edmond/Softwares/commonroad/commonroad-reachable-set/commonroad_reachset/tests"


@pytest.fixture
def node():
    """Provides a Node with square polygons for both directions"""
    return ReachNode(None, None, 0)


@pytest.fixture
def dict_config(path_config):
    """Provides a config dict from the default and scenario-specific configs"""

    ConfigurationBuilder.set_path_to_config(path_config)
    ConfigurationBuilder.build_configuration("DEU_IV21-1_1_T-1")
    dict_config = ConfigurationBuilder.dict_config_overridden

    return dict_config


@pytest.fixture
def config(path_config):
    """Provides a Configuration from the default and scenario-specific configs"""

    ConfigurationBuilder.set_path_to_config(path_config)
    config = ConfigurationBuilder.build_configuration("DEU_IV21-1_1_T-1")

    return config


@pytest.fixture
def continuous_reachability_analysis(config):
    """Provides a ContinuousReachabilityAnalysis object from the given config"""
    reachability_analysis = ContinuousReachabilityAnalysis(config)

    return reachability_analysis


@pytest.fixture
def list_polygons_lon():
    """Provides a list of longitudinal polygons"""
    list_polygons_lon = [ReachPolygon([(2, 2), (6.3, 3.2), (12.7, 7.5), (8.3, 8.3), (3.7, 4.5)])]

    return list_polygons_lon


@pytest.fixture
def list_polygons_lat():
    """Provides a list of lateral polygons"""
    list_polygons_lat = [ReachPolygon([(-2.1, 1.7), (6.3, 3.2), (4.1, 7.5), (0.7, 5.5)])]

    return list_polygons_lat


@pytest.fixture
def list_rectangles_discritized():
    list_rectangles_discritized = [ReachPolygon.from_rectangle_vertices(1, 3, 16, 21)]

    return list_rectangles_discritized


@pytest.fixture
def collision_checker(path_config):
    ConfigurationBuilder.set_path_to_config(path_config)
    config = ConfigurationBuilder.build_configuration("DEU_Test-1_1_T-1")

    return CollisionChecker(config)
