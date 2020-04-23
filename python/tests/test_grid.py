
import sys
import numpy
import time

import ctrigrid_bindings # this needs to be copied in the local directory


s=0.5
cube_vertices=[
        -s, -s, -s,
         s, -s, -s,
         s,  s, -s,
        -s,  s, -s,
        -s, -s,  s,
         s, -s,  s,
         s,  s,  s,
        -s,  s,  s,
        ]
x=0.577350269
cube_normals=[
        -x, -x, -x,
         x, -x, -x,
         x,  x, -x,
        -x,  x, -x,
        -x, -x,  x,
         x, -x,  x,
         x,  x,  x,
        -x,  x,  x,
        ]
cube_indices=[
        0, 1, 2, 2, 3, 0,
        0, 4, 5, 5, 1, 0,
        1, 5, 6, 6, 2, 1,
        2, 6, 7, 7, 3, 2,
        3, 7, 4, 4, 0, 3,
        4, 7, 6, 6, 5, 4,
]


def gen_random_points(count, origin, steps, width):

    min3 = origin
    max3 = ctrigrid_bindings.vec3(
            origin.x + steps * width,
            origin.y + steps * width,
            origin.z + steps * width)

    x = numpy.random.rand(count, 1)
    y = numpy.random.rand(count, 1)
    z = numpy.random.rand(count, 1)
    
    cx = (min3.x + max3.x) / 2
    cy = (min3.y + max3.y) / 2
    cz = (min3.z + max3.z) / 2

    lx = min3.x - max3.x
    ly = min3.y - max3.y
    lz = min3.z - max3.z

    x = (x - 1/2) * lx + cx
    y = (y - 1/2) * ly + cy
    z = (z - 1/2) * lz + cz

    p = numpy.hstack((x, y, z))
    return p.ravel().tolist()

def main():

    mesh_vertices = cube_vertices
    mesh_indices = cube_indices
    mesh_normals = cube_normals

    # create the grid
    origin = ctrigrid_bindings.vec3(-1.0, -1.0, -1.0)
    N = 16
    width = 1/8
    grid = ctrigrid_bindings.grid(N, N, N, width, origin)
    grid.add_tri_mesh(mesh_vertices, mesh_indices)

    # query closest point
    c = gen_random_points(1024, origin, N, width)

    start = time.perf_counter()
    cp, _ = grid.closest_points(c)   # note that with small number of points, multiple threads is not faster
    end = time.perf_counter()
    print("Grid query finished in {} msecs".format((end - start) * 1000))

    print(cp)


if __name__ == "__main__":
    main()
