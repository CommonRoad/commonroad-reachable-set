#cython: infer_types=True
# from collision import CollisionChecker
import ctypes
from libc.math cimport round as round_c
import numpy as np
# cdef rasterize(bool[:,:] occ_grid, CollisionChecker cc, float ll_x, float ll_y, float, ur_x, float ur_y, dx, dy):
cimport cython

@cython.boundscheck(False)  # Deactivate bounds checking
@cython.wraparound(False)   # Deactivate negative indexing.
cpdef convert_cart2pixel_coordinates_c(double[::,::1] cart, double ll_x, double ll_y, double dx_div, double dy_div, Py_ssize_t n):
    # cdef Py_ssize_t n = cart.shape[0]
    cdef int[::, ::1] result = np.empty([n,2], dtype=np.intc)

    # x and y are switched for openCV!
    cdef Py_ssize_t i
    for i in range(n):
        result[i,0] = <int>round_c((cart[i,1] - ll_y) * dy_div)

    for i in range(n):
        result[i,1] = <int>round_c((cart[i,0] - ll_x) * dx_div)

    return result

cpdef get_vertices_from_rect(double[:] center, float r_x, float r_y, double[:] locx, double[:] locy):

    cdef double r_x2[2]
    r_x2[0] = locx[0] * r_x
    r_x2[1] = locx[1] * r_x

    cdef double r_y2[2]
    r_y2[0] = locy[0] * r_y
    r_y2[1] = locy[1] * r_y
    cdef double[:,:] vertices = np.zeros([4,2])

    vertices[0,0] = center[0] + r_x2[0] + r_y2[0]
    vertices[0,1] = center[1] + r_x2[1] + r_y2[1]
    vertices[1,0] = center[0] - r_x2[0] + r_y2[0]
    vertices[1,1] = center[1] - r_x2[1] + r_y2[1]
    vertices[2,0] = center[0] - r_x2[0] - r_y2[0]
    vertices[2,1] = center[1] - r_x2[1] - r_y2[1]
    vertices[3,0] = center[0] + r_x2[0] - r_y2[0]
    vertices[3,1] = center[1] + r_x2[1] - r_y2[1]
    return vertices
