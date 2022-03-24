#include "test_utility.hpp"

TEST_SUITE("TestReachPolygon") {
TEST_CASE("creating polygons with less than three vertices throws exception") {
    vector<tuple<double, double>> vec_vertices = {{0,  0},
                                                  {10, 10}};
    CHECK_THROWS(ReachPolygon{vec_vertices});
}
// todo: add Gerald's test from Python
TEST_CASE("intersect halfspace") {
    vector<tuple<double, double>> vec_vertices = {{10, 0},
                                                  {30, 0},
                                                  {30, 20,},
                                                  {10, 20}};
    auto p = ReachPolygon{vec_vertices};

    // first cut
    p.intersect_halfspace(1, -1, 20);
    vector<tuple<double, double>> vec_vertices_expected = {{20, 0},
                                                           {30, 10}};
    for (auto& vertex_expected: vec_vertices_expected) {
        CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
    }

    // second cut
    p.intersect_halfspace(0.5, 1, 25);
    vec_vertices_expected = {{10, 20}};
    for (auto& vertex_expected: vec_vertices_expected) {
        CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
    }

    // third cut
    p.intersect_halfspace(-2, -1, -30);
    vec_vertices_expected = {{10, 10},
                             {15, 0}};
    for (auto& vertex_expected: vec_vertices_expected) {
        CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
    }

    // final result
    vec_vertices_expected = {{15, 0},
                             {20, 0},
                             {30, 10},
                             {10, 20},
                             {10, 10}};
    for (auto& vertex_expected: vec_vertices_expected) {
        CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
    }
}

TEST_CASE("linear mapping") {
    vector<tuple<double, double>> vec_vertices = {{10, 0},
                                                  {30, 0},
                                                  {30, 20,},
                                                  {10, 20}};
    auto polygon = ReachPolygon{vec_vertices};

    polygon.linear_mapping(1, 0.1, 0, 1);

    vector<tuple<double, double>> vec_vertices_expected = {{10, 0},
                                                           {30, 0},
                                                           {32, 20},
                                                           {12, 20}};
    for (auto& vertex_expected: vec_vertices_expected) {
        CHECK(vertex_in_vertices(vertex_expected, polygon.vertices()));
    }
}
}