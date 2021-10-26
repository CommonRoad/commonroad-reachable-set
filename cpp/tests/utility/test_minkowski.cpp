#include "test_utility.hpp"

TEST_SUITE("TestMinkowski") {
TEST_CASE("minkowski sum returns correct result") {
    SUBCASE("test case 1") {
        vector<tuple<double, double>> vec_vertices = {{-1, -1},
                                                      {1,  -1},
                                                      {0,  1}};
        auto poly1 = make_shared<ReachPolygon>(vec_vertices);
        poly1->convexify();

        vec_vertices.clear();
        vec_vertices = {{3, -1},
                        {5, -1},
                        {5, 1,},
                        {3, 1}};
        auto poly2 = ReachPolygon(vec_vertices);

        poly2.minkowski_sum(poly1);
        poly2.convexify();

        vector<tuple<double, double>> vec_vertices_expected = {{2, -2},
                                                               {6, -2},
                                                               {6, 0},
                                                               {5, 2},
                                                               {3, 2},
                                                               {2, 0},
                                                               {2, -2}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, poly2.vertices()));
        }
    }

    SUBCASE("test case 2") {
        vector<tuple<double, double>> vec_vertices = {{10, 0},
                                                      {30, 0},
                                                      {70, 20,},
                                                      {50, 20}};
        auto poly1 = make_shared<ReachPolygon>(vec_vertices);
        poly1->convexify();

        vec_vertices.clear();
        vec_vertices = {{4,  4},
                        {4,  2},
                        {0,  -2,},
                        {-4, -4},
                        {-4, -2},
                        {0,  2}};
        auto poly2 = ReachPolygon(vec_vertices);
        poly2.convexify();

        poly2.minkowski_sum(poly1);

        vector<tuple<double, double>> vec_vertices_expected = {{6,  -4},
                                                               {26, -4},
                                                               {70, 18.},
                                                               {74, 22.},
                                                               {74, 24.},
                                                               {54, 24.},
                                                               {10, 2.},
                                                               {6,  -2}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, poly2.vertices()));
        }
    }
}
}