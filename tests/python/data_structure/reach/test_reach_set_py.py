import numpy as np

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_set_py import PyReachableSet


def test_reachability_analysis_initialization(config: Configuration):
    PyReachableSet(config)
    assert True


def test_initial_drivable_area(reachable_set_py):
    [polygon_drivable_area] = reachable_set_py._construct_initial_drivable_area()

    # values from the scenario DEU_Test-1_1_T-1
    if reachable_set_py.config.planning.coordinate_system == "CART":
        p_lon = 35.1
        p_lat = 2.1

    else:
        p_lon = 35.1
        p_lat = 0.1

    dp = 0.01

    list_vertices_expected = [(p_lon - dp, p_lat - dp), (p_lon + dp, p_lat - dp),
                              (p_lon + dp, p_lat + dp), (p_lon - dp, p_lat + dp)]
    list_vertices_expected = [(round(x, 2), round(y, 2)) for x, y in list_vertices_expected]
    list_vertices_polygon = [(round(x, 2), round(y, 2)) for x, y in polygon_drivable_area.vertices]

    for vertex in list_vertices_expected:
        is_close = False
        for vertex_drivable_area in list_vertices_polygon:
            is_close = is_close or np.allclose(vertex, vertex_drivable_area)

        assert is_close
