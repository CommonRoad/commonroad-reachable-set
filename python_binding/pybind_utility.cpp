#include "pybind.hpp"

#include <boost/polygon/polygon.hpp>
#include <boost/numeric/conversion/cast.hpp>

namespace py = pybind11;
using namespace reach;

void export_utility(py::module& m) {
    m.def("print_vertices_polygon", &print_vertices_polygon);
    m.def("create_curvilinear_collision_checker", &create_curvilinear_collision_checker);
    m.def("print_collision_checker", &print_collision_checker);

    // TODO: Move function body to .cpp
    m.def("connected_reachset_boost", [](py::list reach_set_nodes, int number_of_digits){
      double coeff = pow(10.0, number_of_digits);
      // create connectivity extraction object
      boost::polygon::connectivity_extraction_90<int64_t> ce;

      // iterate over nodes in list of reachable set nodes
      for (const auto &node : reach_set_nodes) {
        reach::ReachNode *n = node.cast<reach::ReachNode *>();
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
      py::dict ret;

      // iterate over elements in map graph
      for(auto elem : map_graph) {
        // only consider overlapping elements from setB
        py::list overlapping_rectangle_ids;
        // iterate over elements in the set (i.e., the second element of each pair in map_graph)
        for(auto set_elem : elem.second) {
          // create tuple for each overlap pair between the node (elem.first) and the nodes in elem.second
          py::tuple adj_pair = py::make_tuple(py::cast(elem.first), py::cast(set_elem));
          overlapping_rectangle_ids.append(adj_pair);
        }
        // add list of tuples to dict for key=elem.first
        ret[py::cast(elem.first)] = overlapping_rectangle_ids;
      }
      return ret;
    }, "Function determines connected sets within given list of reachable set nodes");
}