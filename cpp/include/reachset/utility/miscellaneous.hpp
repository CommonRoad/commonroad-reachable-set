#pragma once

#include <Eigen/Dense>
#include "shared_include.hpp"
#include "reachset/data_structure/configuration.hpp"

#include "collision/collision_checker.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/rectangle_obb.h"
#include "collision/time_variant_collision_object.h"

reach::ConfigurationPtr create_mock_configuration();

collision::TimeVariantCollisionObjectPtr create_tvo_from_specs(
        double const& length, double const& width, tuple<double, double> const& p_init, double const& v_init,
        double const& o_init, int const& time_steps);