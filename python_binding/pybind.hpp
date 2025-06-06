#pragma once

#include <nanobind/nanobind.h>

#include "reachset/utility/shared_include.hpp"
#include "reachset/utility/shared_using.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"

#include "reachset/data_structure/reach/reach_node.hpp"
#include "reachset/data_structure/configuration.hpp"

#include "reachset/data_structure/reach/reach_set.hpp"

#include "reachset/utility/collision_checker.hpp"

void export_data_structures(nanobind::module_& m);

void export_utility(nanobind::module_& m);

void export_reach(nanobind::module_& m);

// ----

void export_configuration(nanobind::module_& m);

void export_reach_polygon(nanobind::module_& m);

void export_reach_node(nanobind::module_& m);

void export_reachable_set(nanobind::module_& m);
