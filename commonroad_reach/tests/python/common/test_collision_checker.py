from commonroad_reach.data_structure.collision_checker_cpp import CppCollisionChecker
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


def test_convert_reach_polygon_to_collision_object(collision_checker: CppCollisionChecker):
    tuple_vertices = (60, 2, 65, 6)
    rect_input = ReachPolygon.from_rectangle_vertices(*tuple_vertices)

    rect_output = collision_checker.convert_reach_polygon_to_collision_object(rect_input)

    assert (
               rect_output.min_x(),
               rect_output.min_y(),
               rect_output.max_x(),
               rect_output.max_y(),
           ) == tuple_vertices

# def test_check_collision_and_split_rectangles(collision_checker: CollisionChecker):
#     list_rectangles = [ReachPolygon.from_rectangle_vertices(60, 2, 65, 6)]
#     list_tuples_coords_rectangles_expected = [
#         (60.0, 2.0, 61.25, 4.0),
#         (60.0, 4.0, 61.25, 6.0),
#         (61.25, 4.5, 61.5625, 5.0),
#         (61.25, 5.0, 62.5, 6.0),
#         (62.5, 5.0, 62.8125, 5.5),
#         (62.5, 5.5, 63.125, 6.0),
#         (63.125, 5.5, 63.75, 6.0),
#         (63.75, 5.5, 64.375, 6.0),
#     ]
#
#     list_rectangles_collision_free = check_collision_and_split_rectangles(
#         collision_checker=collision_checker,
#         time_step=0,
#         list_rectangles=list_rectangles,
#         radius_terminal_split=1.0,
#     )
#
#     for rectangle in list_rectangles_collision_free:
#         # print(rectangle.bounds)
#         assert rectangle.bounds in list_tuples_coords_rectangles_expected
