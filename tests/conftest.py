"""Shared fixtures"""
import pathlib

import pytest

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_set_py import PyReachableSet


@pytest.fixture
def config():
    config = _open_config("DEU_Test-1_1_T-1")
    config.general.path_offline_data = str(pathlib.Path(__file__).parent.joinpath("precomputed_offline_data").resolve())
    return config


@pytest.fixture
def config_offline():
    return _open_config("DEU_Offline-1_1_T-1")


def _open_config(name_scenario: str) -> Configuration:
    path_root = str(pathlib.Path(__file__).parent.resolve())
    config = ConfigurationBuilder(path_root=path_root).build_configuration(name_scenario)
    config.update()
    return config


@pytest.fixture
def collision_checker_cpp(config: Configuration):
    try:
        from commonroad_reach.data_structure.collision_checker import CollisionChecker

    except ImportError:
        print("Importing C++ collision checker failed.")

    else:
        config.vehicle.ego.radius_inflation = config.vehicle.ego.radius_disc
        return CollisionChecker(config)


@pytest.fixture
def node():
    """Provides a Node with square polygons for both directions"""
    return ReachNode(None, None, 0)


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
def reachable_set_py(config):
    return PyReachableSet(config)
