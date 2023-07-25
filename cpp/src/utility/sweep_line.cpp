#include "reachset/utility/sweep_line.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;

SweepLine::Event::Event(EventType const& event_type, double const& p_lon,
                        double const& p_lat_low, double const& p_lat_high) :
        type(event_type), p_lon(p_lon),
        p_lat_low(p_lat_low), p_lat_high(p_lat_high) {}

bool SweepLine::Event::operator==(const Event& other) const {
    if (this->type == other.type and
        this->p_lon == other.p_lon and
        this->p_lat_low == other.p_lat_low and
        this->p_lat_high == other.p_lat_high)
        return true;

    return false;
}

/// *Steps*:
/// 1. create a segment tree with min/max lateral position of rectangles
/// 2. create events of line sweeping: left = ENTER, right = EXIT
/// 3. create vertical segments with events
vector<ReachLinePtr>
SweepLine::obtain_vertical_segments_from_rectangles(vector<ReachPolygonPtr> const& vec_rectangles) {
    if (vec_rectangles.empty())
        return {};

    // Step 1
    auto[p_lat_min_rectangles, p_lat_max_rectangles] = compute_extremum_lateral_positions_of_rectangles(vec_rectangles);
    SweepLine::tree_counter = CounterSegmentTree(p_lat_min_rectangles, p_lat_max_rectangles);

    // Step 2
    auto vec_events = SweepLine::create_events(vec_rectangles);

    // Step 3
    auto vec_segments_vertical = create_vertical_segments_from_events(vec_events);

    return vec_segments_vertical;
}

tuple<double, double>
SweepLine::compute_extremum_lateral_positions_of_rectangles(vector<ReachPolygonPtr> const& vec_rectangles) {
    auto p_lat_min_rectangles = std::numeric_limits<double>::infinity();
    auto p_lat_max_rectangles = -std::numeric_limits<double>::infinity();

    for (auto const& rectangle: vec_rectangles) {
        p_lat_min_rectangles = std::min(p_lat_min_rectangles, rectangle->p_lat_min());
        p_lat_max_rectangles = std::max(p_lat_max_rectangles, rectangle->p_lat_max());
    }

    return {p_lat_min_rectangles, p_lat_max_rectangles};
}

vector<EventPtr> SweepLine::create_events(vector<ReachPolygonPtr> const& vec_rectangles) {
    vector<EventPtr> vec_events;
    vec_events.reserve(vec_rectangles.size());

    for (auto const& rectangle: vec_rectangles) {
        vec_events.emplace_back(make_shared<Event>(EventType::ENTER,
                                                   rectangle->p_lon_min(),
                                                   rectangle->p_lat_min(),
                                                   rectangle->p_lat_max()));

        vec_events.emplace_back(make_shared<Event>(EventType::EXIT,
                                                   rectangle->p_lon_max(),
                                                   rectangle->p_lat_min(),
                                                   rectangle->p_lat_max()));
    }

    sort_events(vec_events);

    return vec_events;
}

/// Comparisons are made in sequence on:
/// 1. longitudinal position of rectangles
/// 2. type of the event
/// 3. lower lateral position
void SweepLine::sort_events(std::vector<EventPtr>& vec_events) {
    std::sort(vec_events.begin(), vec_events.end(), [](EventPtr const& event_a, EventPtr const& event_b) {
        if (event_a->p_lon < event_b->p_lon)
            return true;
        else if (event_a->p_lon > event_b->p_lon)
            return false;

        if (event_a->type == EventType::ENTER and event_b->type == EventType::EXIT)
            return true;
        else if (event_a->type == EventType::EXIT and event_b->type == EventType::ENTER)
            return false;

        if (event_a->p_lat_low < event_b->p_lat_low)
            return true;
        else if (event_a->p_lat_low > event_b->p_lat_low)
            return false;

        return false;
    });
}

/// For each event, query the tree to get the nonactive intervals, which is the desired vertical segment.
vector<ReachLinePtr> SweepLine::create_vertical_segments_from_events(vector<EventPtr> const& vec_events) {
    vector<ReachLinePtr> vec_segments_vertical;

    for (auto const& event: vec_events) {
        if (event->type == EventType::ENTER) {
            auto vec_segments_from_event = create_vertical_segments_from_event(event);
            vec_segments_vertical.insert(vec_segments_vertical.end(),
                                         std::make_move_iterator(vec_segments_from_event.begin()),
                                         std::make_move_iterator(vec_segments_from_event.end()));

            tree_counter.activate(event->p_lat_low, event->p_lat_high);
        } else {
            tree_counter.deactivate(event->p_lat_low, event->p_lat_high);

            auto vec_segments_from_event = create_vertical_segments_from_event(event);
            vec_segments_vertical.insert(vec_segments_vertical.end(),
                                         std::make_move_iterator(vec_segments_from_event.begin()),
                                         std::make_move_iterator(vec_segments_from_event.end()));
        }
    }
    return vec_segments_vertical;
}

vector<ReachLinePtr> SweepLine::create_vertical_segments_from_event(EventPtr const& event) {
    vector<ReachLinePtr> vec_segments;

    auto stack_intervals_lat = SweepLine::tree_counter.get_stack_of_non_active_intervals(event->p_lat_low,
                                                                                         event->p_lat_high);

    while (not stack_intervals_lat.empty()) {
        auto p_lat_high = stack_intervals_lat.top();
        stack_intervals_lat.pop();
        auto p_lat_low = stack_intervals_lat.top();
        stack_intervals_lat.pop();
        auto p_lon = event->p_lon;

        vec_segments.emplace_back(make_shared<ReachLine>(p_lon, p_lat_low, p_lon, p_lat_high));
    }

    return vec_segments;
}

/// *Steps*:
/// 1. Create a segment tree with the list of segments.
/// 2. Create a dictionary that maps p_lon to a list of rectangles whose left edge is aligned with p_lon.
/// 3. Merge rectangles that share the same coordinates of of p_lat.
vector<ReachPolygonPtr>
SweepLine::create_rectangles_from_vertical_segments(vector<ReachLinePtr> const& vec_segments) {
    // Step 1
    auto[p_lat_min_segments, p_lat_max_segments] = compute_extremum_lateral_positions_of_segments(vec_segments);
    SweepLine::tree_toggle = ToggleSegmentTree(p_lat_min_segments, p_lat_max_segments);

    // Step 2
    auto map_p_lon_to_vec_rectangles = create_p_lon_to_rectangles_map(vec_segments);

    // Step 3
    auto vec_rectangles_final = merge_rectangles_with_same_lateral_coordinates(map_p_lon_to_vec_rectangles);

    return vec_rectangles_final;
}

tuple<double, double>
SweepLine::compute_extremum_lateral_positions_of_segments(vector<ReachLinePtr> const& vec_segments) {
    auto p_lat_min_segments = std::numeric_limits<double>::infinity();
    auto p_lat_max_segments = -std::numeric_limits<double>::infinity();

    for (auto const& segment: vec_segments) {
        p_lat_min_segments = std::min(p_lat_min_segments, segment->p_lat_min);
        p_lat_max_segments = std::max(p_lat_max_segments, segment->p_lat_max);
    }

    return {p_lat_min_segments, p_lat_max_segments};
}

/// *Steps*:
/// 1. Create a dictionary that maps p_lon to list of tuples of p_lat from segments
/// 2. Iterate through p_lon, retrieve relevant tuples of p_lat and toggle the status of segments between these p_lat
/// in the segment tree. Get intervals of active segments and create rectangles.
map<double, vector<ReachPolygonPtr>>
SweepLine::create_p_lon_to_rectangles_map(vector<ReachLinePtr> const& vec_segments) {
    if (vec_segments.empty())
        return {};

    map<double, vector<ReachPolygonPtr>> map_p_lon_to_vec_rectangles;

    // Step 1
    map<double, vector<tuple<double, double>>> map_p_lon_to_vec_tuples_p_lat;
    vector<double> vec_p_lon;
    vec_p_lon.reserve(vec_segments.size());

    for (auto const& segment: vec_segments) {
        map_p_lon_to_vec_tuples_p_lat[segment->p_lon_min].emplace_back(
                make_tuple(segment->p_lat_min, segment->p_lat_max));
        vec_p_lon.emplace_back(segment->p_lon_min);
    }
    vec_p_lon.erase(unique(vec_p_lon.begin(), vec_p_lon.end()), vec_p_lon.end());

    // Step 2
    for (int i = 0; i < vec_p_lon.size() - 1; ++i) {
        auto p_lon_min = vec_p_lon[i];
        auto p_lon_max = vec_p_lon[i + 1];
        auto vec_tuples_p_lat = map_p_lon_to_vec_tuples_p_lat[p_lon_min];

        for (auto const&[p_lat_min, p_lat_max]: vec_tuples_p_lat)
            tree_toggle.toggle(p_lat_min, p_lat_max);

        auto stack_intervals_active = tree_toggle.get_stack_of_active_intervals();
        while (not stack_intervals_active.empty()) {
            auto p_lat_max = stack_intervals_active.top();
            stack_intervals_active.pop();
            auto p_lat_min = stack_intervals_active.top();
            stack_intervals_active.pop();

            map_p_lon_to_vec_rectangles[p_lon_min].emplace_back(
                    make_shared<ReachPolygon>(p_lon_min, p_lat_min, p_lon_max, p_lat_max));
        }

    }

    return map_p_lon_to_vec_rectangles;
}

/// Iterate through pairs of lists of rectangles, if there is a right rectangle with the same lateral coordinates,
/// then do not add to list. Instead, the right rectangle is popped and replaced by the merged one.
vector<ReachPolygonPtr> SweepLine::merge_rectangles_with_same_lateral_coordinates(
        map<double, vector<ReachPolygonPtr>>& map_p_lon_to_vec_rectangles) {

    vector<ReachPolygonPtr> vec_rectangles_merged;

    vector<double> vec_p_lon;
    vec_p_lon.reserve(map_p_lon_to_vec_rectangles.size());
    for (auto const&[key, val]: map_p_lon_to_vec_rectangles) {
        vec_p_lon.emplace_back(key);
    }

    for (int i = 0; i < vec_p_lon.size() - 1; ++i) {
        auto p_lon_left = vec_p_lon[i];
        auto p_lon_right = vec_p_lon[i + 1];

        auto& vec_rectangles_left = map_p_lon_to_vec_rectangles[p_lon_left];
        auto& vec_rectangles_right = map_p_lon_to_vec_rectangles[p_lon_right];

        for (auto const& rectangle_left: vec_rectangles_left) {
            auto add_to_list = true;

            for (auto const& rectangle_right: vec_rectangles_right) {
                if (rectangles_have_same_p_lat(rectangle_left, rectangle_right) &&
                    rectangle_left->p_lon_max() >= rectangle_right->p_lon_min()) {
                    auto it_end = std::remove(vec_rectangles_right.begin(), vec_rectangles_right.end(),
                                              rectangle_right);
                    vec_rectangles_right.erase(it_end, vec_rectangles_right.end());

                    vec_rectangles_right.emplace_back(make_shared<ReachPolygon>(rectangle_left->p_lon_min(),
                                                                                rectangle_left->p_lat_min(),
                                                                                rectangle_right->p_lon_max(),
                                                                                rectangle_left->p_lat_max()));
                    add_to_list = false;
                    break;
                }
            }

            if (add_to_list)
                vec_rectangles_merged.emplace_back(rectangle_left);
        }
    }
    // add rectangles from the last vector into final vector
    for (auto const& rectangle: map_p_lon_to_vec_rectangles[vec_p_lon.back()]) {
        vec_rectangles_merged.emplace_back(rectangle);
    }

    return vec_rectangles_merged;
}

bool SweepLine::rectangles_have_same_p_lat(ReachPolygonPtr const& rectangle1, ReachPolygonPtr const& rectangle2) {
    return rectangle1->p_lat_min() == rectangle2->p_lat_min() and rectangle1->p_lat_max() == rectangle2->p_lat_max();
}