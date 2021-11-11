#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "reachset/utility/shared_include.hpp"
#include "reachset/utility/shared_using.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"

#include "reachset/data_structure/reach/reach_node.hpp"
#include "reachset/data_structure/configuration.hpp"

#include "reachset/data_structure/reach/reach_interface.hpp"
#include "reachset/data_structure/reach/reach_analysis.hpp"

#include "reachset/data_structure/collision_checker.hpp"

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
