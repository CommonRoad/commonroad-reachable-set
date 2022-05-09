#pragma once

#include "shared_include.hpp"
#include "reachset/data_structure/reach/reach_line.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reachset/data_structure/segment_tree.hpp"

namespace reach {
struct SweepLine {
    /// Possible types of events in the sweep line algorithm.
    enum class EventType {
        /// Entering (left edge)
        ENTER,
        /// Exiting (right edge)
        EXIT
    };

    /// Event in the sweep line algorithm.
    /// p_lon = x (CART) = s (CLCS), p_lat = y (CART) = d (CLCS)\n
    /// It is assumed that the line is swept from left to right. The left edge of a rectangle is typed 'ENTER',
    /// and the right edge 'EXIT'.
    struct Event {
        EventType type;
        double p_lon;
        double p_lat_low;
        double p_lat_high;

        Event(EventType const& event_type, double const& p_lon, double const& p_lat_low, double const& p_lat_high);

        bool operator==(Event const& other) const;
    };

    inline static CounterSegmentTree tree_counter = CounterSegmentTree();
    inline static ToggleSegmentTree tree_toggle = ToggleSegmentTree();

    /// Returns a vector of vertical segments for the input rectangles.
    static std::vector<ReachLinePtr>
    obtain_vertical_segments_from_rectangles(std::vector<ReachPolygonPtr> const& vec_rectangles);

    static std::tuple<double, double>
    compute_extremum_lateral_positions_of_rectangles(std::vector<ReachPolygonPtr> const& vec_rectangles);

    /// Creates a vector of sorted events with the given vector of rectangles.
    static std::vector<std::shared_ptr<Event>> create_events(std::vector<ReachPolygonPtr> const& vec_rectangles);

    /// Sorts events.
    static void sort_events(std::vector<std::shared_ptr<SweepLine::Event>>& vector);

    /// Creates a vector of vertical segments from vector of events.
    static std::vector<ReachLinePtr>
    create_vertical_segments_from_events(std::vector<std::shared_ptr<Event>> const& vec_events);

    /// Returns a vector of vertical segments with the tree and event.
    static std::vector<ReachLinePtr> create_vertical_segments_from_event(std::shared_ptr<Event> const& event);

    /// Returns a vector of rectangles from the given vertical segments.
    static std::vector<ReachPolygonPtr>
    create_rectangles_from_vertical_segments(std::vector<ReachLinePtr> const& vec_rectangles);

    /// Computes the extremum lateral positions of a vector of segments.
    static std::tuple<double, double>
    compute_extremum_lateral_positions_of_segments(std::vector<ReachLinePtr> const& vec_segments);

    /// Create a map that maps p_lon to a vector of rectangles whose left edge is aligned with p_lon.
    static std::map<double, std::vector<ReachPolygonPtr>>
    create_p_lon_to_rectangles_map(std::vector<ReachLinePtr> const& vec_segments);

    /// Return a vector of rectangles with possible merging.
    static std::vector<ReachPolygonPtr> merge_rectangles_with_same_lateral_coordinates(
            std::map<double, std::vector<ReachPolygonPtr>>& map_p_lon_to_vec_rectangles);

    /// Returns true of the two rectangles have the same lateral positions.
    static bool rectangles_have_same_p_lat(ReachPolygonPtr const& rectangle1, ReachPolygonPtr const& rectangle2);
};

using EventPtr = std::shared_ptr<SweepLine::Event>;
}
