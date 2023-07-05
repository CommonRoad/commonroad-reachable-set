#include "test_utility.hpp"

using namespace reach;

TEST_SUITE("TestSweepLine") {
TEST_CASE("compute extremum lateral positions of rectangles") {
    auto vec_rectangles = {make_shared<ReachPolygon>(1, 1, 3, 3),
                           make_shared<ReachPolygon>(2, 2, 4, 4)};

    auto p_lat_min_expected = 1;
    auto p_lat_max_expected = 4;

    auto[p_lat_min, p_lat_max] = SweepLine::compute_extremum_lateral_positions_of_rectangles(vec_rectangles);

    CHECK(p_lat_min == p_lat_min_expected);
    CHECK(p_lat_max == p_lat_max_expected);
}

TEST_CASE("create event list") {
    auto vec_rectangles = {make_shared<ReachPolygon>(1, 1, 3, 3),
                           make_shared<ReachPolygon>(2, 2, 4, 4)};

    auto vec_events_expected = {
            SweepLine::Event(SweepLine::EventType::ENTER, 1, 1, 3),
            SweepLine::Event(SweepLine::EventType::EXIT, 3, 1, 3),
            SweepLine::Event(SweepLine::EventType::ENTER, 2, 2, 4),
            SweepLine::Event(SweepLine::EventType::EXIT, 4, 2, 4)
    };

    auto vec_events = SweepLine::create_events(vec_rectangles);
    vector<SweepLine::Event> vec_events_unwrapped;
    vec_events_unwrapped.reserve(vec_events.size());
    for (auto const& event: vec_events)
        vec_events_unwrapped.emplace_back(*event);

    for (auto const& event: vec_events_expected) {
        CHECK(event_in_events(event, vec_events_expected));
    }
}

TEST_CASE("sort events") {
    vector<EventPtr> vec_events = {
            make_shared<SweepLine::Event>(SweepLine::EventType::EXIT, 5, 5, 10),
            make_shared<SweepLine::Event>(SweepLine::EventType::EXIT, 3, 2, 10),
            make_shared<SweepLine::Event>(SweepLine::EventType::EXIT, 10, 2, 10),
            make_shared<SweepLine::Event>(SweepLine::EventType::ENTER, 10, 3, 10),
            make_shared<SweepLine::Event>(SweepLine::EventType::ENTER, 10, 2, 10)
    };

    SweepLine::sort_events(vec_events);

    CHECK(vec_events[0]->p_lon == 3);
    CHECK(vec_events[1]->p_lon == 5);
    CHECK(vec_events[2]->type == SweepLine::EventType::ENTER);
    CHECK(vec_events[3]->p_lat_low == 3);
}

TEST_CASE("obtain vertical segments from rectangles returns correct segments") {
    SUBCASE("test case 1") {
        auto vec_rectangles = {make_shared<ReachPolygon>(1, 1, 3, 3),
                               make_shared<ReachPolygon>(2, 2, 4, 4)};

        auto vec_vertices_segments_expected = {
                ReachLine(1.0, 1.0, 1.0, 3.0),
                ReachLine(2.0, 3.0, 2.0, 4.0),
                ReachLine(3.0, 1.0, 3.0, 2.0),
                ReachLine(4.0, 2.0, 4.0, 4.0)
        };

        auto vec_segments_vertical = SweepLine::obtain_vertical_segments_from_rectangles(vec_rectangles);
        vector<ReachLine> vec_segments_unwrapped;
        vec_segments_unwrapped.reserve(vec_segments_vertical.size());
        for (auto const& segment: vec_segments_vertical)
            vec_segments_unwrapped.emplace_back(*segment);

        for (auto const& segment: vec_segments_unwrapped) {
            CHECK(segment_in_segments(segment, vec_vertices_segments_expected));
        }
    }

    SUBCASE("test case 2") {
        auto vec_rectangles = {make_shared<ReachPolygon>(4, 0, 8, 4),
                               make_shared<ReachPolygon>(4, 6, 8, 10),
                               make_shared<ReachPolygon>(1, 3, 5, 7),
                               make_shared<ReachPolygon>(7, 3, 11, 7)};

        auto vec_vertices_segments_expected = {
                ReachLine(1.0, 3.0, 1.0, 7.0),
                ReachLine(4.0, 0.0, 4.0, 3.0),
                ReachLine(4.0, 7.0, 4.0, 10.0),
                ReachLine(5.0, 4.0, 5.0, 6.0),
                ReachLine(7.0, 4.0, 7.0, 6.0),
                ReachLine(8.0, 0.0, 8.0, 3.0),
                ReachLine(8.0, 7.0, 8.0, 10.0),
                ReachLine(11.0, 3.0, 11.0, 7.0)
        };

        auto vec_segments_vertical = SweepLine::obtain_vertical_segments_from_rectangles(vec_rectangles);
        vector<ReachLine> vec_segments_unwrapped;
        vec_segments_unwrapped.reserve(vec_segments_vertical.size());
        for (auto const& segment: vec_segments_vertical)
            vec_segments_unwrapped.emplace_back(*segment);

        for (auto const& segment: vec_segments_unwrapped) {
            CHECK(segment_in_segments(segment, vec_vertices_segments_expected));
        }
    }
}

TEST_CASE("merge rectangles with same lateral coordinates") {
    map<double, vector<ReachPolygonPtr>> map_p_lon_to_vec_rectangles;
    map_p_lon_to_vec_rectangles[0].emplace_back(make_shared<ReachPolygon>(0, 0, 1, 1));
    map_p_lon_to_vec_rectangles[1].emplace_back(make_shared<ReachPolygon>(1, 0, 2, 1));
    map_p_lon_to_vec_rectangles[2].emplace_back(make_shared<ReachPolygon>(2, 0, 3, 1));
    map_p_lon_to_vec_rectangles[3].emplace_back(make_shared<ReachPolygon>(3, 0, 4, 1));
    map_p_lon_to_vec_rectangles[4].emplace_back(make_shared<ReachPolygon>(4, 0, 6, 1));

    auto vec_rectangles_merged = SweepLine::merge_rectangles_with_same_lateral_coordinates(map_p_lon_to_vec_rectangles);

    REQUIRE_EQ(vec_rectangles_merged.size(), 1);
    CHECK_EQ(vec_rectangles_merged[0]->bounding_box(), make_tuple(0, 0, 6, 1));
}

TEST_CASE("merge rectangles with same lateral coordinates disconnected") {
    map<double, vector<ReachPolygonPtr>> map_p_lon_to_vec_rectangles;
    map_p_lon_to_vec_rectangles[0].emplace_back(make_shared<ReachPolygon>(0, 0, 1, 1));
    map_p_lon_to_vec_rectangles[2].emplace_back(make_shared<ReachPolygon>(2, 0, 3, 1));

    auto vec_rectangles_merged = SweepLine::merge_rectangles_with_same_lateral_coordinates(map_p_lon_to_vec_rectangles);

    REQUIRE_EQ(vec_rectangles_merged.size(), 2);
    CHECK_EQ(vec_rectangles_merged[0]->bounding_box(), make_tuple(0, 0, 1, 1));
    CHECK_EQ(vec_rectangles_merged[1]->bounding_box(), make_tuple(2, 0, 3, 1));
}

TEST_CASE("create partitioned rectangles from vertical segments") {
    auto vec_rectangles = {make_shared<ReachPolygon>(4, 0, 8, 4),
                           make_shared<ReachPolygon>(4, 6, 8, 10),
                           make_shared<ReachPolygon>(1, 3, 5, 7),
                           make_shared<ReachPolygon>(7, 3, 11, 7)};

    vector<tuple<double, double, double, double>> vec_bounds_rectangles_expected = {{1.0, 3.0, 4.0,  7.0},
                                                                                    {4.0, 0.0, 5.0,  10.0},
                                                                                    {5.0, 6.0, 7.0,  10.0},
                                                                                    {5.0, 0.0, 7.0,  4.0},
                                                                                    {7.0, 0.0, 8.0,  10.0},
                                                                                    {8.0, 3.0, 11.0, 7.0}};

    auto vec_segments_vertical = SweepLine::obtain_vertical_segments_from_rectangles(vec_rectangles);
    auto vec_rectangles_partitioned = SweepLine::create_rectangles_from_vertical_segments(vec_segments_vertical);

    for (auto const& rectangle: vec_rectangles_partitioned)
        CHECK(bound_in_bounds(rectangle->bounding_box(), vec_bounds_rectangles_expected));
}
}