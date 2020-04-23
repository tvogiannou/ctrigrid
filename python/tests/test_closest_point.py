
import sys
import numpy

import ctrigrid_bindings # this needs to be copied in the local directory


def computeNormal(v0, v1, v2):
    v = numpy.array([
        [v0.x, v0.y, v0.z],
        [v1.x, v1.y, v1.z],
        [v2.x, v2.y, v2.z]])

    e0 = v[1] - v[0]
    e1 = v[2] - v[1]
    n = numpy.cross(e0, e1)

    return n

def main():
    v0 = ctrigrid_bindings.vec3(0.5, 0.0, 0.2)
    v1 = ctrigrid_bindings.vec3(0.0, 1.0, 0.0)
    v2 = ctrigrid_bindings.vec3(-0.5, 0.0, -0.1)

    p = ctrigrid_bindings.vec3(0.9, 0.9, 0.3)
    c = ctrigrid_bindings.closest_point_tri(v0, v1, v2, p)

    p1 = ctrigrid_bindings.vec3(-0.9, 0.4, 0.3)
    c1 = ctrigrid_bindings.closest_point_tri(v0, v1, v2, p1)
    
    p2 = ctrigrid_bindings.vec3(0.0, 0.5, 0.4)
    c2 = ctrigrid_bindings.closest_point_tri(v0, v1, v2, p2)

    print(c.x, c.y, c.z)
    print(c1.x, c1.y, c1.z)
    print(c2.x, c2.y, c2.z)



if __name__ == "__main__":
    main()
