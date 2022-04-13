#include "test_utility.hpp"

TEST_SUITE("TestReachPolygon") {
TEST_CASE("creating polygons with less than three vertices throws exception") {
    vector<tuple<double, double>> vec_vertices = {{0,  0},
                                                  {10, 10}};
    CHECK_THROWS(ReachPolygon{vec_vertices});
}

TEST_CASE("has correct bounding box upon initialization") {
    vector<tuple<double, double>> vec_vertices = {{0,  0},
                                                  {5,  -5},
                                                  {10, 10}};
    auto polygon = ReachPolygon{vec_vertices};
    CHECK(polygon.bounding_box() == tuple(0, -5, 10, 10));
}

TEST_CASE("has correct bounding box after inserting new vertex") {
    vector<tuple<double, double>> vec_vertices = {{0,  0},
                                                  {5,  -5},
                                                  {10, 10}};
    auto polygon = ReachPolygon{vec_vertices};
    polygon.add_vertex(20, -10);
    CHECK(polygon.bounding_box() == tuple(0, -10, 20, 10));
}

TEST_CASE("convexification") {
    vector<tuple<double, double>> vec_vertices = {{0,  0},
                                                  {5,  -5},
                                                  {10, 10},
                                                  {5,  2},
                                                  {5,  -2}};
    auto polygon = ReachPolygon{vec_vertices};
    polygon.convexify();

    vector<tuple<double, double>> vec_vertices_expected = {{0,  0},
                                                           {5,  -5},
                                                           {10, 10}};
    for (auto& vertex_expected: vec_vertices_expected) {
        CHECK(vertex_in_vertices(vertex_expected, polygon.vertices()));
    }
    CHECK(polygon.vertices().size() == 3);
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

TEST_CASE("minkowski sum") {
    SUBCASE("test case 1") {
        vector<tuple<double, double>> vec_vertices = {{-1, -1},
                                                      {1,  -1},
                                                      {0,  1}};
        auto poly1 = make_shared<ReachPolygon>(vec_vertices);
        poly1->convexify();

        vec_vertices = {{3, -1},
                        {5, -1},
                        {5, 1,},
                        {3, 1}};
        auto poly2 = ReachPolygon(vec_vertices);
        poly2.convexify();

        poly2.minkowski_sum(poly1);

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

TEST_CASE("intersect halfspace - general case") {
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

TEST_CASE("intersect halfspace - special case") {
    vector<tuple<double, double>> vec_vertices = {{10,  -10},
                                                  {10,  10},
                                                  {-10, 10,},
                                                  {-10, -10}};
    auto p = ReachPolygon{vec_vertices};

    SUBCASE("vertical halfspace: ax <= c, b = 0 and a > 0") {
        // first cut
        p.intersect_halfspace(2, 0, 10);
        vector<tuple<double, double>> vec_vertices_expected = {{5,   -10},
                                                               {-10, -10},
                                                               {-10, 10},
                                                               {5,   10}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }

        // second cut
        p.intersect_halfspace(2, 0, 0);
        vec_vertices_expected = {{0,   -10},
                                 {-10, -10},
                                 {-10, 10},
                                 {0,   10}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }

        // third cut
        p.intersect_halfspace(2, 0, -10);
        vec_vertices_expected = {{-5,  -10},
                                 {-10, -10},
                                 {-10, 10},
                                 {-5,  10}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }
    }

    SUBCASE("vertical halfspace: ax <= c, b = 0 and a < 0") {
        // first cut
        p.intersect_halfspace(-2, 0, 10);
        vector<tuple<double, double>> vec_vertices_expected = {{10, -10},
                                                               {-5, -10},
                                                               {-5, 10},
                                                               {10, 10}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }

        // second cut
        p.intersect_halfspace(-2, 0, 0);
        vec_vertices_expected = {{10, -10},
                                 {0,  -10},
                                 {0,  10},
                                 {10, 10}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }

        // third cut
        p.intersect_halfspace(-2, 0, -10);
        vec_vertices_expected = {{10, -10},
                                 {5,  -10},
                                 {5,  10},
                                 {10, 10}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }
    }

    SUBCASE("horizontal halfspace: by <= c, a = 0 and b > 0") {
        // first cut
        p.intersect_halfspace(0, 2, 10);
        vector<tuple<double, double>> vec_vertices_expected = {{-10, -10},
                                                               {-10, 5},
                                                               {-10, 5},
                                                               {10,  -10}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }

        // second cut
        p.intersect_halfspace(0, 2, 0);
        vec_vertices_expected = {{-10, -10},
                                 {-10, 0},
                                 {10,  0,},
                                 {10,  -10}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }

        // third cut
        p.intersect_halfspace(0, 2, -10);
        vec_vertices_expected = {{-10, -10},
                                 {-10, -5},
                                 {10,  -5,},
                                 {10,  -10}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }
    }

    SUBCASE("horizontal halfspace: by <= c, a = 0 and b < 0") {
        // first cut
        p.intersect_halfspace(0, -2, 10);
        vector<tuple<double, double>> vec_vertices_expected = {{-10, -5},
                                                               {-10, 10},
                                                               {10,  10},
                                                               {10,  -5}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }

        // second cut
        p.intersect_halfspace(0, -2, 0);
        vec_vertices_expected = {{-10, 0},
                                 {-10, 10},
                                 {10,  10},
                                 {10,  0}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }

        // third cut
        p.intersect_halfspace(0, -2, -10);
        vec_vertices_expected = {{-10, 5},
                                 {-10, 10},
                                 {10,  10},
                                 {10,  5}};
        for (auto& vertex_expected: vec_vertices_expected) {
            CHECK(vertex_in_vertices(vertex_expected, p.vertices()));
        }
    }
}
}