import pytest

from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility.sweep_line import SweepLine, Event, EventType


def test_compute_extremum_lateral_positions_of_rectangles():
    list_rectangles = [ReachPolygon([(1, 1), (3, 1), (3, 3), (3, 1)]),
                       ReachPolygon([(2, 2), (4, 2), (4, 4), (4, 2)])]
    p_lat_min_expected = 1
    p_lat_max_expected = 4

    p_lat_min_rectangles, p_lat_max_rectangles \
        = SweepLine.compute_extremum_lateral_positions_of_rectangles(list_rectangles)

    assert p_lat_min_rectangles == p_lat_min_expected and p_lat_max_rectangles == p_lat_max_expected


def test_sort_events():
    list_events = [Event(EventType.EXIT, 5, 5, 10),
                   Event(EventType.EXIT, 3, 2, 10),
                   Event(EventType.EXIT, 10, 2, 10),
                   Event(EventType.ENTER, 10, 3, 10),
                   Event(EventType.ENTER, 10, 2, 10)]

    list_events = SweepLine.sort_events(list_events)

    assert list_events[0].p_lon == 3 and \
           list_events[1].p_lon == 5 and \
           list_events[2].type == EventType.ENTER and \
           list_events[3].p_lat_low == 3


def test_create_event_list():
    list_rectangles = [ReachPolygon([(1, 1), (3, 1), (3, 3), (1, 3)]), ReachPolygon([(2, 2), (4, 2), (4, 4), (2, 4)])]
    list_events_expected = [Event(EventType.ENTER, 1, 1, 3),
                            Event(EventType.EXIT, 3, 1, 3),
                            Event(EventType.ENTER, 2, 2, 4),
                            Event(EventType.EXIT, 4, 2, 4)]

    list_events = SweepLine.create_event_list(list_rectangles)

    for event in list_events_expected:
        assert event in list_events


@pytest.mark.parametrize(
    "list_tuples_vertices, list_vertices_segments_expected",
    [
        ([(1, 1, 3, 3), (2, 2, 4, 4)],
         [(1.0, 1.0, 1.0, 3.0),
          (2.0, 3.0, 2.0, 4.0),
          (3.0, 1.0, 3.0, 2.0),
          (4.0, 2.0, 4.0, 4.0)]),

        ([(4, 0, 8, 4), (4, 6, 8, 10), (1, 3, 5, 7), (7, 3, 11, 7)],
         [(1.0, 3.0, 1.0, 7.0),
          (4.0, 0.0, 4.0, 3.0),
          (4.0, 7.0, 4.0, 10.0),
          (5.0, 4.0, 5.0, 6.0),
          (7.0, 4.0, 7.0, 6.0),
          (8.0, 0.0, 8.0, 3.0),
          (8.0, 7.0, 8.0, 10.0),
          (11.0, 3.0, 11.0, 7.0)
          ])
    ]
)
def test_obtain_vertical_segments_from_list_of_rectangles_returns_correct_segments(
        list_tuples_vertices, list_vertices_segments_expected):
    list_rectangles = [
        ReachPolygon.from_rectangle_vertices(*tuple_coords) for tuple_coords in list_tuples_vertices]

    def function_sort(x):
        return x.bounds[0]

    list_segments_vertical = SweepLine.obtain_vertical_segments_from_rectangles(list_rectangles)

    for segment in list_segments_vertical:
        assert segment.bounds in list_vertices_segments_expected

    # test if p_lon_min is less than p_lon_max
    for segment in list_segments_vertical:
        assert segment.p_lon_min <= segment.p_lon_max

    # test if the obtained segments are already sorted with their p_lon_min values
    assert list_segments_vertical == sorted(list_segments_vertical, key=function_sort)


def test_create_partitioned_rectangles_from_vertical_segments():
    list_tuples_vertices = [(4, 0, 8, 4), (4, 6, 8, 10), (1, 3, 5, 7), (7, 3, 11, 7)]
    list_tuples_vertices_rectangles_expected = [(1.0, 3.0, 4.0, 7.0),
                                                (4.0, 0.0, 5.0, 10.0),
                                                (5.0, 6.0, 7.0, 10.0),
                                                (5.0, 0.0, 7.0, 4.0),
                                                (7.0, 0.0, 8.0, 10.0),
                                                (8.0, 3.0, 11.0, 7.0)]

    list_rectangles = [ReachPolygon.from_rectangle_vertices(*tuple_coords) for tuple_coords in list_tuples_vertices]

    list_segments_vertical = SweepLine.obtain_vertical_segments_from_rectangles(list_rectangles)

    list_rectangles_partitioned = SweepLine.create_rectangles_from_vertical_segments(list_segments_vertical)

    for rectangle in list_rectangles_partitioned:
        assert rectangle.bounds in list_tuples_vertices_rectangles_expected


def test_merge_rectangles_with_same_lateral_coordinates():
    dict_p_lon_to_list_rectangles = {0: [ReachPolygon.from_rectangle_vertices(0, 0, 1, 1)],
                                     1: [ReachPolygon.from_rectangle_vertices(1, 0, 2, 1)],
                                     2: [ReachPolygon.from_rectangle_vertices(2, 0, 3, 1)],
                                     3: [ReachPolygon.from_rectangle_vertices(3, 0, 4, 1)],
                                     4: [ReachPolygon.from_rectangle_vertices(4, 0, 6, 1)]}

    list_rectangles_merged = SweepLine.merge_rectangles_with_same_lateral_coordinates(dict_p_lon_to_list_rectangles)

    assert len(list_rectangles_merged) == 1
    assert list_rectangles_merged[0].bounds == (0, 0, 6, 1)


def test_merge_rectangles_with_same_lateral_coordinates_disconnected():
    # we must not merge rectangles with same lateral coordinates, but gap in the longitudinal coordinates
    dict_p_lon_to_list_rectangles = {0: [ReachPolygon.from_rectangle_vertices(0, 0, 1, 1)],
                                     2: [ReachPolygon.from_rectangle_vertices(2, 0, 3, 1)]}

    list_rectangles_merged = SweepLine.merge_rectangles_with_same_lateral_coordinates(dict_p_lon_to_list_rectangles)

    assert len(list_rectangles_merged) == 2
    assert list_rectangles_merged[0].bounds == (0, 0, 1, 1)
    assert list_rectangles_merged[1].bounds == (2, 0, 3, 1)
