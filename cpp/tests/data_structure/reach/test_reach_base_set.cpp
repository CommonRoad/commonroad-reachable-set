#include "test_utility.hpp"

TEST_SUITE("TestReachBaseSet") {
TEST_CASE("position rectangle and boundaries") {
    auto polygon_lon = ReachPolygon::from_rectangle_coordinates(0, 0, 5, 10);
    auto polygon_lat = ReachPolygon::from_rectangle_coordinates(7, -5, 15, 5);
    auto base_set = ReachBaseSet{polygon_lon, polygon_lat};

    SUBCASE("has correct rectangle bounding box") {
        CHECK(base_set.position_rectangle()->bounding_box() == std::make_tuple(0, 7, 5, 15));
    }

    SUBCASE("has correct vertices") {
        CHECK(base_set.p_lon_min() == 0);
        CHECK(base_set.p_lon_max() == 5);
        CHECK(base_set.v_lon_min() == 0);
        CHECK(base_set.v_lon_max() == 10);
        CHECK(base_set.p_lat_min() == 7);
        CHECK(base_set.p_lat_max() == 15);
        CHECK(base_set.v_lat_min() == -5);
        CHECK(base_set.v_lat_max() == 5);
    }
}
}