import numpy as np
import pytest

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.utility import configuration as util_configuration

from commonroad.common.solution import VehicleType
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping


def test_construct_configuration(config: Configuration):
    assert config.planning.coordinate_system == "CVLN"


@pytest.mark.parametrize("length, width, wheelbase, radius_expected",
                         [(6, 2, None, np.sqrt(2)), (0, 2, 4, 1.0), (4.927, 2.208, None, 1.37591)])
def test_compute_disk_radius_and_wheelbase(length, width, wheelbase, radius_expected):
    _radius, _wheelbase = util_configuration.compute_disc_radius_and_wheelbase(length, width, wheelbase)

    assert np.isclose(_radius, radius_expected)

    # if the wheelbase is not given, check if it is correctly computed
    if not wheelbase:
        assert np.isclose(_wheelbase, length / 3 * 2)


@pytest.mark.parametrize("id_type_vehicle, reference_point, rad_expected, dist_expected",
                         [(1, "CENTER", 1.10168, 2.86533),  (1, "REAR", 1.15260, 3.01752),
                          (2, "CENTER", 1.10114, 3.00533), (2, "REAR", 1.15717, 2.84543),
                          (3, "CENTER", 1.19581, 3.046), (3, "REAR", 1.33347, 2.64227)])
def test_compute_disc_radius_and_distance(id_type_vehicle, reference_point, rad_expected, dist_expected):

    # get vehicle params from CR vehicle models
    vehicle_params = VehicleParameterMapping.from_vehicle_type(VehicleType(id_type_vehicle))

    # compute radius and circle distance
    _rad, _dist = util_configuration.compute_disc_radius_and_distance\
        (vehicle_params.l, vehicle_params.w, ref_point=reference_point, dist_axle_rear=vehicle_params.b)

    assert np.isclose(_rad, rad_expected)
    assert np.isclose(_dist, dist_expected)
