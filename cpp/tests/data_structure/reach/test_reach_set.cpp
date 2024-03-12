#include "test_utility.hpp"

TEST_SUITE("TestContinuousReachabilityAnalysis") {
TEST_CASE("initialization") {
    auto config = Configuration::load_configuration("configurations/test_cpp.yaml");
    auto reach_set = ReachableSet(config);
    auto drivable_area = reach_set.drivable_area_at_step(0);

    SUBCASE("has correct initial drivable area") {
        auto p_lon_min = -0.01;
        auto p_lon_max = 0.01;
        auto p_lat_min = -0.01;
        auto p_lat_max = 0.01;

        vector<tuple<double, double>> vec_vertices_expected = {{p_lon_min, p_lat_min},
                                                               {p_lon_min, p_lat_max},
                                                               {p_lon_max, p_lat_min},
                                                               {p_lon_max, p_lat_max}};

        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, drivable_area[0]->vertices()));
        }
    }
}
}