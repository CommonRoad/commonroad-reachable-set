from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility.reach_operation import check_collision_and_split_rectangles


def test_convert_reach_polygon_to_collision_object(collision_checker_cpp):
    tuple_vertices = (60, 2, 65, 6)
    rect_input = ReachPolygon.from_rectangle_vertices(*tuple_vertices)

    rect_output = collision_checker_cpp.convert_reach_polygon_to_collision_object(rect_input)

    assert (rect_output.min_x(), rect_output.min_y(), rect_output.max_x(), rect_output.max_y()) == tuple_vertices


def test_check_collision_and_split_rectangles_cpp(collision_checker_cpp):
    list_rectangles = [ReachPolygon.from_rectangle_vertices(60, 2, 65, 6)]
    list_tuples_coords_rectangles_expected = [(60.0, 2.0, 61.25, 4.0),
                                              (61.25, 3.5, 61.875, 4.0),
                                              (61.875, 3.5, 62.5, 4.0),
                                              (60.0, 4.0, 60.625, 4.5),
                                              (60.625, 4.0, 61.25, 4.5),
                                              (61.25, 4.0, 61.875, 4.5),
                                              (61.875, 4.0, 62.5, 4.5),
                                              (62.5, 3.5, 63.125, 4.0),
                                              (63.125, 3.5, 63.75, 4.0),
                                              (63.75, 3.5, 64.375, 4.0),
                                              (64.375, 3.5, 65.0, 4.0),
                                              (62.5, 4.0, 63.125, 4.5),
                                              (63.125, 4.0, 63.75, 4.5),
                                              (63.75, 4.0, 64.375, 4.5),
                                              (64.375, 4.0, 65.0, 4.5)]

    list_rectangles_collision_free = check_collision_and_split_rectangles(collision_checker=collision_checker_cpp,
                                                                          step=0, list_rectangles=list_rectangles,
                                                                          radius_terminal_split=1.0)

    for rectangle in list_rectangles_collision_free:
        assert rectangle.bounds in list_tuples_coords_rectangles_expected
