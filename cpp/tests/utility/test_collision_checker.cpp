#include "test_utility.hpp"

TEST_SUITE("TestCollisionCheckerUtility") {
TEST_CASE("create aabb from_coordinates") {
    auto aabb = create_aabb_from_coordinates(0.0, 0.0, 20.0, 10.0);

    CHECK(aabb->r_x() == 10.0);
    CHECK(aabb->r_y() == 5.0);
    CHECK(aabb->center_x() == 10.0);
    CHECK(aabb->center_y() == 5.0);
}

TEST_CASE("obtain extremum coordinates of polyline") {
    EigenPolyline polyline;
    polyline.emplace_back(Eigen::Vector2d(0.0, 0.0));
    polyline.emplace_back(Eigen::Vector2d(1.0, 1.0));
    polyline.emplace_back(Eigen::Vector2d(2.0, -2.0));
    polyline.emplace_back(Eigen::Vector2d(3.0, 3.0));
    polyline.emplace_back(Eigen::Vector2d(4.0, 5.0));

    auto[p_lon_min, p_lat_min, p_lon_max, p_lat_max] =
            obtain_extremum_coordinates_of_polyline(polyline);

    CHECK(p_lon_min == 0.0);
    CHECK(p_lat_min == -2.0);
    CHECK(p_lon_max == 4.0);
    CHECK(p_lat_max == 5.0);
}
}