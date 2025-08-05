#include "test_utility.hpp"

TEST_SUITE("TestReachOperation") {
TEST_CASE("bounding polygon has correct vertices") {
    auto polygon_bounding = create_bounding_box(2.0, -5.0, 10.0);

    std::vector<std::tuple<double, double>> vec_vertices_expected = {{-10, -10},
                                                           {20,  -10},
                                                           {-10, 20},
                                                           {20,  20}};
    for (auto& vertex_expected: vec_vertices_expected) {
        CHECK(vertex_in_vertices(vertex_expected, polygon_bounding->vertices()));
    }
}

TEST_CASE("zero-state polygon has correct vertices") {
    auto polygon = create_zero_state_polygon(2.0, -2.0, 2.0);

    std::vector<std::tuple<double, double>> vec_vertices_expected = {{4,  4},
                                                           {-4, -4},
                                                           {0,  2},
                                                           {0,  -2},
                                                           {-4, -2},
                                                           {4,  2}};
    for (auto& vertex_expected: vec_vertices_expected) {
        CHECK(vertex_in_vertices(vertex_expected, polygon->vertices()));
    }
}

TEST_CASE("propagate polygon returns correct vertices") {
    auto config = Configuration::load_configuration("configurations/test_cpp.yaml");
    config->planning().dt = 2.0;
    config->vehicle().ego.v_lon_min = 0;
    config->vehicle().ego.v_lon_max = 20;
    config->vehicle().ego.a_lon_min = -2.0;
    config->vehicle().ego.a_lon_max = 2.0;

    config->vehicle().ego.v_lat_min = 0;
    config->vehicle().ego.v_lat_max = 20;
    config->vehicle().ego.a_lat_min = -2.0;
    config->vehicle().ego.a_lat_max = 2.0;

    std::vector<std::tuple<double, double>> vec_vertices = {{10, 0},
                                                  {30, 0},
                                                  {30, 20,},
                                                  {10, 20}};
    auto polygon_lon = std::make_shared<ReachPolygon>(vec_vertices);
    auto reach_set = ReachableSet(config);

    auto polygon_lon_propagated = propagate_polygon(polygon_lon,
                                                    reach_set.polygon_zero_state_lon,
                                                    config->planning().dt,
                                                    config->vehicle().ego.v_lon_min,
                                                    config->vehicle().ego.v_lon_max);


    std::vector<std::tuple<double, double>> vec_vertices_expected = {{72, 20},
                                                           {70, 18},
                                                           {34, 0},
                                                           {8,  0},
                                                           {10, 2},
                                                           {46, 20}};


    for (auto& vertex_expected: vec_vertices_expected) {
        CHECK(vertex_in_vertices(vertex_expected, polygon_lon_propagated->vertices()));
    }
}

TEST_CASE("compute minimum positions of polygons") {
    std::vector<ReachPolygonPtr> vec_base_sets_propagated{std::make_shared<ReachPolygon>(1, 1, 5, 5),
                                                     std::make_shared<ReachPolygon>(-5, 5, 10, 10)};

    auto[p_lon_min, p_lat_min] = compute_minimum_positions_of_rectangles(vec_base_sets_propagated);

    CHECK(p_lon_min == -5);
    CHECK(p_lat_min == 1);
}

TEST_CASE("discretize position rectangles") {
    std::vector<std::tuple<double, double>> vec_vertices = {{2, 2},
                                                  {6.3, 3.2},
                                                  {12.7, 7.5},
                                                  {8.3, 8.3},
                                                  {3.7, 4.5}};
    std::vector<ReachPolygonPtr> vec_rectangle = {std::make_shared<ReachPolygon>(vec_vertices)};
    auto tuple_p_min = compute_minimum_positions_of_rectangles(vec_rectangle);

    SUBCASE("size_grid = 0.5") {
        auto vec_rectangles_discretized = discretize_rectangles(vec_rectangle, tuple_p_min, 0.5);
        auto rectangle = vec_rectangles_discretized[0];

        std::tuple<double, double, double, double> tuple_coords_expected = {0.0, 0.0, 22.0, 13.0};
        CHECK(std::get<0>(tuple_coords_expected) == std::get<0>(rectangle->bounding_box()));
        CHECK(std::get<1>(tuple_coords_expected) == std::get<1>(rectangle->bounding_box()));
        CHECK(std::get<2>(tuple_coords_expected) == std::get<2>(rectangle->bounding_box()));
        CHECK(std::get<3>(tuple_coords_expected) == std::get<3>(rectangle->bounding_box()));
    }

    SUBCASE("size_grid = 0.5") {
        auto vec_rectangles_discretized = discretize_rectangles(vec_rectangle, tuple_p_min, 0.2);
        auto rectangle = vec_rectangles_discretized[0];

        std::tuple<double, double, double, double> tuple_coords_expected = {0.0, 0.0, 54.0, 32.0};
        CHECK(std::get<0>(tuple_coords_expected) == std::get<0>(rectangle->bounding_box()));
        CHECK(std::get<1>(tuple_coords_expected) == std::get<1>(rectangle->bounding_box()));
        CHECK(std::get<2>(tuple_coords_expected) == std::get<2>(rectangle->bounding_box()));
        CHECK(std::get<3>(tuple_coords_expected) == std::get<3>(rectangle->bounding_box()));
    }

}

TEST_CASE("undiscretize position rectangles") {
    std::vector<ReachPolygonPtr> vec_rectangle = {std::make_shared<ReachPolygon>(0, 0, 22, 13)};
    std::tuple<double, double> tuple_p_min = {3, 3};

    SUBCASE("size_grid = 0.5") {
        auto vec_rectangles_undiscretized = undiscretize_rectangles(vec_rectangle, tuple_p_min, 0.5);
        auto rectangle = vec_rectangles_undiscretized[0];

        std::tuple<double, double, double, double> tuple_coords_expected = {3.0, 3.0, 14.0, 9.5};
        CHECK(std::get<0>(tuple_coords_expected) == std::get<0>(rectangle->bounding_box()));
        CHECK(std::get<1>(tuple_coords_expected) == std::get<1>(rectangle->bounding_box()));
        CHECK(std::get<2>(tuple_coords_expected) == std::get<2>(rectangle->bounding_box()));
        CHECK(std::get<3>(tuple_coords_expected) == std::get<3>(rectangle->bounding_box()));
    }
}

TEST_CASE("creating adjacency dictionary") {
    std::vector<ReachPolygonPtr> vec_rectangles_a = {
            std::make_shared<ReachPolygon>(1, 0, 2, 1),
            std::make_shared<ReachPolygon>(2, 0, 3, 1)};

    std::vector<ReachPolygonPtr> vec_rectangles_b = {
            std::make_shared<ReachPolygon>(0.5, 0.5, 1.5, 1.5),
            std::make_shared<ReachPolygon>(1.5, 0.5, 2.5, 1.5),
            std::make_shared<ReachPolygon>(2.5, 0.5, 3.5, 1.5)};

    auto map_adjacency = create_adjacency_map(vec_rectangles_a, vec_rectangles_b);

    std::unordered_map<int, std::vector<int>> map_adjacency_expected = {{0, {0, 1}},
                                                              {1, {1, 2}}};

    CHECK(map_adjacency == map_adjacency_expected);
}

TEST_CASE("overlapping relationship of rectangles") {
    std::vector<ReachPolygonPtr> vec_rectangles_a = {
            std::make_shared<ReachPolygon>(0.5, 0.5, 1.5, 1.5),
            std::make_shared<ReachPolygon>(1.5, 0.5, 2.5, 1.5)};

    std::vector<ReachPolygonPtr> vec_rectangles_b = {
            std::make_shared<ReachPolygon>(0, 0, 1, 1),
            std::make_shared<ReachPolygon>(1, 0, 2, 1),
            std::make_shared<ReachPolygon>(2, 0, 3, 1)};

    auto map_id_rectangle_a_to_vec_idx_rectangles_b = create_adjacency_map(vec_rectangles_a, vec_rectangles_b);

    CHECK(map_id_rectangle_a_to_vec_idx_rectangles_b[0] == std::vector<int>{0, 1});
    CHECK(map_id_rectangle_a_to_vec_idx_rectangles_b[1] == std::vector<int>{1, 2});
}
}