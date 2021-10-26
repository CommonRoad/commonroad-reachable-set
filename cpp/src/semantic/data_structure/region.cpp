#include "reachset/semantic/data_structure/region.hpp"
#include "reachset/common/utility/shared_using.hpp"

using namespace reach;

Region::Region(pybind11::handle region_py) {
    vec_ids_lanelets = region_py.attr("list_ids_lanelets").cast<std::vector<int >>();
    set_propositions = region_py.attr("set_propositions").cast<set<string >>();

    auto vec_vertices_cart = region_py.attr("polygon_cart").attr("vertices")
            .cast<vector<tuple<double, double>>>();
    polygon_cart = make_shared<ReachPolygon>(vec_vertices_cart);

    auto vec_vertices_cvln = region_py.attr("polygon_cvln").attr("vertices")
            .cast<vector<tuple<double, double>>>();
    polygon_cvln = make_shared<ReachPolygon>(vec_vertices_cvln);
}

bool Region::excludes(ReachPolygonPtr const& rectangle, std::string const& coordinate) const {
    auto[p_lon_min_box, p_lat_min_box, p_lon_max_box, p_lat_max_box] = rectangle->bounding_box();

    double p_lon_min, p_lat_min, p_lon_max, p_lat_max;
    if (coordinate == "CART")
        std::tie(p_lon_min, p_lat_min, p_lon_max, p_lat_max) = polygon_cart->bounding_box();

    else if (coordinate == "CVLN")
        std::tie(p_lon_min, p_lat_min, p_lon_max, p_lat_max) = polygon_cvln->bounding_box();

    else
        throw std::logic_error("<Region> Provided coordinate system is invalid.");

    if (p_lon_max_box < p_lon_min or p_lon_min_box > p_lon_max
        or p_lat_max_box < p_lat_min or p_lat_min_box > p_lat_max)
        return true;
    else return false;
}
