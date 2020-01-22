from math import sqrt
import numpy as np

cdef float distance(float ox, float oy, float x, float y):
    return sqrt((ox - x) ** 2 + (oy - y) ** 2)


def mynorm(a, b=(0.0, 0.0)):
    return distance(a[0], a[1], b[0], b[1])


def is_path_collision_free(x_new, x_old, list obstacle_list):
    # circle and line intersection check
    # https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    d = x_new - x_old
    cdef double ox, oy, r
    cdef double a, b, c
    for (ox, oy, r) in obstacle_list:
        f = x_old - np.array([ox, oy])
        a = 0
        b = 0
        c = 0
        for i in range(len(x_new)):
            a += d[i] * d[i]
            b += f[i] * d[i]
            c += f[i] * f[i]
        b *= 2
        c -= r * r
        discriminant = b * b - 4 * a * c
        if discriminant >= 0 and a > 0:
            if 0 < (-b - sqrt(discriminant)) / (2 * a) <= 1:
                return False
    return True


def safety_cost_func(float x, float y, list obstacle_list, tuple bounds=(1,2,3)):
    cdef int r
    for (ox, oy, ro) in obstacle_list:
        for r in bounds:
            if (ox + x) ** 2 + (oy + y) ** 2 < ((ro + r) ** 2):
                return 4 - r
    return 0