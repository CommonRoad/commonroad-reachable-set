from matplotlib import pyplot as plt
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon as ReachPolygonPy
from commonroad_reach.pycrreach import ReachPolygon as ReachPolygonCpp

from commonroad.geometry.shape import Polygon
from commonroad.visualization.mp_renderer import MPRenderer

# polygon vertices
list_vert_poly = [
    (0.03, 0.6),
    (0, 0.3),  # r
    (-0.03, -0.3),  # OUTSIDE
    (-0.03, -0.6),  # q
    (0, -0.3),
    (0.03, 0.3),
]

# halfspace parameters
a = -1.2
b = 0
c = 0.036

# halfspace line equation
x_min = -0.03
x_max = -0.03
y_min = -0.6
y_max = 0.6


# # halfspace parameters
# a = 0
# b = 1
# c = 0.03
#
# # halfspace line equation
# x_min = -0.03
# x_max = 0.03
# y_min = -a * x_min + c/b
# y_max = -a * x_max + c/b


# halfspace intersection python
reach_poly_py = ReachPolygonPy(list_vert_poly)
reach_poly_py_intersected = reach_poly_py.intersect_halfspace(a, b, c)

# halfspace intersection cpp
reach_poly_cpp = ReachPolygonCpp(list_vert_poly)
reach_poly_cpp.intersect_halfspace(a, b, c)

# plot python
rnd = MPRenderer(plot_limits=[-0.3, 0.3, -0.6, 0.6])
cr_poly_py = Polygon(reach_poly_py.vertices)
cr_poly_py_intersected = Polygon(reach_poly_py_intersected.vertices)
# cr_poly.draw(rnd)
cr_poly_py_intersected.draw(rnd)
rnd.render()
rnd.ax.plot([x_min, x_max], [y_min, y_max], color="black", zorder=100)
plt.show()

# plot cpp
rnd = MPRenderer(plot_limits=[-0.3, 0.3, -0.6, 0.6])
cr_poly_cpp = Polygon(reach_poly_cpp.vertices)
cr_poly_cpp.draw(rnd)
# cr_poly_cpp_intersected.draw(rnd)
rnd.render()
rnd.ax.plot([x_min, x_max], [y_min, y_max], color="black", zorder=100)
plt.show()