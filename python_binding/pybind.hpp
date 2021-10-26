#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "reachset/common/utility/shared_include.hpp"
#include "reachset/common/utility/shared_using.hpp"
#include "reachset/common/data_structure/reach_polygon.hpp"

#include "reachset/common/data_structure/reach_node.hpp"
#include "reachset/common/data_structure/configuration.hpp"

#include "reachset/common/reachable_set_interface.hpp"
#include "reachset/continuous/continuous_reachability_analysis.hpp"

#include "reachset/common/collision_checker.hpp"

namespace py = pybind11;

void export_data_structures(py::module& m);

void export_utility(py::module& m);

void export_reachable_set(py::module& m);

// ----

void export_configuration(py::module& m);

void export_reach_polygon(py::module& m);

void export_reach_node(py::module& m);

void export_reachable_set_interface(py::module& m);

void export_continuous_reachability_analysis(py::module& m);
