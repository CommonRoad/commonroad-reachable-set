import pycrreachset as reach
import time
import random

def main():
    list_vertices_1 = [(10, 0),
                       (70, 20),
                       (30, 0),
                       (50, 20)]

    list_vertices_2 = [(4, 4),
                       (4, 2),
                       (-4, -4),
                       (-4, -2),
                       (0, -2),
                       (0, 2)]

    poly1 = reach.ReachPolygon(list_vertices_1)
    poly2 = reach.ReachPolygon(list_vertices_2)
    poly1.convexify()
    poly2.convexify()

    poly1.minkowski_sum_naive(poly2)

    print(poly1.vertices())

    poly1 = reach.ReachPolygon(list_vertices_1)
    poly2 = reach.ReachPolygon(list_vertices_2)
    poly1.convexify()
    poly2.convexify()

    print(poly1.vertices())
    print(poly2.vertices())
    poly1.minkowski_sum_stefanie(poly2)

    print(poly1.vertices())


# def main():
#     num_runs = 500
#     num_vertices_max = 50
#
#     time_sum_naive = 0
#     time_sum_stefanie = 0
#     time_sum_cgal_1 = 0
#     time_sum_cgal_2 = 0
#     time_sum_cgal_3 = 0
#     for _ in range(num_runs):
#         # == naive
#         num_vertices_1 = random.randint(3, num_vertices_max)
#         num_vertices_2 = random.randint(3, num_vertices_max)
#
#         list_vertices_1 = [(random.random() * 50, random.random() * 50) for _ in range(num_vertices_1)]
#         list_vertices_2 = [(random.random() * 50, random.random() * 50) for _ in range(num_vertices_2)]
#
#         poly1 = reach.ReachPolygon(list_vertices_1)
#         poly2 = reach.ReachPolygon(list_vertices_2)
#         poly1.convexify()
#         poly2.convexify()
#
#         time_start = time.process_time()
#         poly1.minkowski_sum_naive(poly2)
#         time_sum_naive += (time.process_time() - time_start)
#
#         # == stefanie
#         num_vertices_1 = random.randint(3, num_vertices_max)
#         num_vertices_2 = random.randint(3, num_vertices_max)
#
#         list_vertices_1 = [(random.random() * 50, random.random() * 50) for _ in range(num_vertices_1)]
#         list_vertices_2 = [(random.random() * 50, random.random() * 50) for _ in range(num_vertices_2)]
#
#         poly1 = reach.ReachPolygon(list_vertices_1)
#         poly2 = reach.ReachPolygon(list_vertices_2)
#         poly1.convexify()
#         poly2.convexify()
#
#         time_start = time.process_time()
#         poly1.minkowski_sum_stefanie(poly2)
#         time_sum_stefanie += (time.process_time() - time_start)
#
#         # == cgal_1
#         num_vertices_1 = random.randint(3, num_vertices_max)
#         num_vertices_2 = random.randint(3, num_vertices_max)
#
#         list_vertices_1 = [(random.randint(0, 50), random.randint(0, 50)) for _ in range(num_vertices_1)]
#         list_vertices_2 = [(random.randint(0, 50), random.randint(0, 50)) for _ in range(num_vertices_2)]
#
#         poly1 = reach.ReachPolygon(list_vertices_1)
#         poly2 = reach.ReachPolygon(list_vertices_2)
#         poly1.convexify()
#         poly2.convexify()
#
#         time_start = time.process_time()
#         poly1.minkowski_sum_cgal1(poly2)
#         time_sum_cgal_1 += (time.process_time() - time_start)
#
#         # # == cgal_2
#         # num_vertices_1 = random.randint(3, num_vertices_max)
#         # num_vertices_2 = random.randint(3, num_vertices_max)
#         #
#         # list_vertices_1 = [(random.random() * 50, random.random() * 50) for _ in range(num_vertices_1)]
#         # list_vertices_2 = [(random.random() * 50, random.random() * 50) for _ in range(num_vertices_2)]
#         #
#         # poly1 = reach.ReachPolygon(list_vertices_1)
#         # poly2 = reach.ReachPolygon(list_vertices_2)
#         # poly1.convexify()
#         # poly2.convexify()
#         #
#         # time_start = time.process_time()
#         # poly1.minkowski_sum_cgal2(poly2)
#         # time_sum_cgal_2 += (time.process_time() - time_start)
#         #
#         # # == cgal_3
#         # num_vertices_1 = random.randint(3, num_vertices_max)
#         # num_vertices_2 = random.randint(3, num_vertices_max)
#         #
#         # list_vertices_1 = [(random.random() * 50, random.random() * 50) for _ in range(num_vertices_1)]
#         # list_vertices_2 = [(random.random() * 50, random.random() * 50) for _ in range(num_vertices_2)]
#         #
#         # poly1 = reach.ReachPolygon(list_vertices_1)
#         # poly2 = reach.ReachPolygon(list_vertices_2)
#         # poly1.convexify()
#         # poly2.convexify()
#         #
#         # time_start = time.process_time()
#         # poly1.minkowski_sum_cgal3(poly2)
#         # time_sum_cgal_3 += (time.process_time() - time_start)
#
#     print(f"Time (naive): {time_sum_naive} s")
#     print(f"Time (stefanie): {time_sum_stefanie} s")
#     print(f"Time (cgal_1): {time_sum_cgal_1} s")
#     print(f"Time (cgal_2): {time_sum_cgal_2} s")
#     print(f"Time (cgal_3): {time_sum_cgal_3} s")
#     print("Done.")


if __name__ == "__main__":
    main()
