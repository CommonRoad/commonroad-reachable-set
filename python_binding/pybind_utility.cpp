#include "pybind.hpp"
#include "reachset/utility/lut_longitudinal_enlargement.hpp"

#include <boost/polygon/polygon.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nanobind::literals;
using namespace reach;

void export_utility(nb::module_& m) {
    m.def("print_vertices_polygon", &print_vertices_polygon);
    m.def("create_curvilinear_collision_checker", &create_curvilinear_collision_checker);
    m.def("create_cartesian_collision_checker", &create_cartesian_collision_checker);
    m.def("print_collision_checker", &print_collision_checker);

    // TODO: Move function body to .cpp
    m.def("connected_reachset_boost", [](nb::list reach_set_nodes, int number_of_digits){
      double coeff = pow(10.0, number_of_digits);
      // create connectivity extraction object
      boost::polygon::connectivity_extraction_90<int64_t> ce;

      // iterate over nodes in list of reachable set nodes
      for (const auto &node : reach_set_nodes) {
        reach::ReachNode *n = nb::cast<reach::ReachNode *>(node);
        // convert reachable set node to boost rectangle and insert as a graph node into connectivity extractor
        ce.insert(boost::polygon::rectangle_data<int64_t>(
            boost::numeric_cast<int64_t>(floor(n->p_lon_min() * coeff)),
            boost::numeric_cast<int64_t>(floor(n->p_lat_min() * coeff)),
            boost::numeric_cast<int64_t>(ceil(n->p_lon_max() * coeff)),
            boost::numeric_cast<int64_t>(ceil(n->p_lat_max() * coeff))));
      }

      // create map type graph and populate the graph with edge data
      std::map<int, std::set<int> > map_graph;
      ce.extract(map_graph);

      // python return dict
      nb::dict ret;

      // iterate over elements in map graph
      for(auto elem : map_graph) {
        // only consider overlapping elements from setB
        nb::list overlapping_rectangle_ids;
        // iterate over elements in the set (i.e., the second element of each pair in map_graph)
        for(auto set_elem : elem.second) {
          // create tuple for each overlap pair between the node (elem.first) and the nodes in elem.second
          nb::tuple adj_pair = nb::make_tuple(nb::cast(elem.first), nb::cast(set_elem));
          overlapping_rectangle_ids.append(adj_pair);
        }
        // add list of tuples to dict for key=elem.first
        ret[nb::cast(elem.first)] = overlapping_rectangle_ids;
      }
      return ret;
    }, "Function determines connected sets within given list of reachable set nodes");

    // TODO: Move function body to .cpp
    m.def("create_adjacency_dictionary_boost", [](nb::list reach_polygons_1, nb::list reach_polygons_2){
      // python return dict
      nb::dict ret;
      // counters
      int i,j;
      // loop over polygons in first list
      i=0;
      for (const auto &poly1: reach_polygons_1) {
        reach::ReachPolygon *p1 = nb::cast<reach::ReachPolygon *>(poly1);
        // initialize list to store IDs of overlapping rectangles
        nb::list overlapping_rectangle_ids;
        // loop over polygons in second list
        j=0;
        for (const auto &poly2: reach_polygons_2){
            std::shared_ptr<reach::ReachPolygon> p2 = nb::cast<std::shared_ptr<reach::ReachPolygon>>(poly2);
            // check for intersection
            if (p1->intersects(p2)) {
                overlapping_rectangle_ids.append(nb::cast(j));
            }
            j++;
        }
        // add adjacency to return dictionary
        ret[nb::cast(i)] = overlapping_rectangle_ids;
        i++;
      }
      return ret;

    }, "Function returns adjacency dictionary with overlapping polygons of two given lists of polygons"
       "E.g.: {0 : [1, 2]} means that ReachPolygon 0 from first list overlaps with ReachPolygons 1,2 from second list");

    nb::class_<reach::LUTLongitudinalEnlargement>(
             m, "LUTLongitudinalEnlargement")
      .def("__init__", [](reach::LUTLongitudinalEnlargement *obj, std::map<double, std::map<std::pair<double, double>, double>> lut) {
        new (obj) reach::LUTLongitudinalEnlargement(lut);
      });
}