
import sys
import time
import numpy

import ctrigrid_bindings # this needs to be copied in the local directory

# generate count random points starting from origin up to lengths
# origin is expected to be of type ctrigrid_bindings.vec3
# lengths is a double[3] array 
# returns a (flattened) list of points 
def gen_random_points(count, origin, lengths):
    min3 = origin
    max3 = ctrigrid_bindings.vec3(
            origin.x + lengths[0],
            origin.y + lengths[1],
            origin.z + lengths[2])

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

# generate grid params for testing
# output params define a grid that will fit the axis aligned 
# bounding box of the vertices expanded by padding_perc, and
# the axes with largest extends will have up to N cells
def gen_grid_test_params(vertices, N, padding_perc=0.1):
    import math
    
    v = vertices.reshape((-1,3))
    v_min = numpy.min(v, axis=0)
    v_max = numpy.max(v, axis=0)
    
    dv = v_max - v_min
    s = numpy.max(dv)

    s_exp = (1+padding_perc) * s
    width = s_exp / (N - 1)

    origin = v_min - 0.5 * padding_perc * numpy.array([s, s, s])

    Nx = math.ceil(N * (1 + padding_perc) * dv[0] / s_exp)
    Ny = math.ceil(N * (1 + padding_perc) * dv[1] / s_exp)
    Nz = math.ceil(N * (1 + padding_perc) * dv[2] / s_exp)

    return Nx, Ny, Nz, width, ctrigrid_bindings.vec3(origin[0], origin[1], origin[2])


def main(filename, N=16, num_test_points=16, verbose=True, padding_perc=0.1):

    mesh_vertices, mesh_indices = ctrigrid_bindings.load_obj(filename)
    if verbose:
        print("Loaded mesh {} with {} vertices and {} triangles".format(filename, int(mesh_vertices.size/3), int(mesh_indices.size/3)))

    # align mesh to origin
    # m = numpy.mean(mesh_vertices.reshape((-1, 3)), axis=0)
    # mesh_vertices = (mesh_vertices.reshape((-1, 3)) - m).ravel()

    Nx, Ny, Nz, width, origin = gen_grid_test_params(mesh_vertices, N, padding_perc)
    if verbose:
        print("Generated grid params\n(Nx, Ny, Nz): ({}, {}, {})\nwidth: {}\norigin: [{}, {}, {}]".format(\
                                        Nx, Ny, Nz, width, origin.x, origin.y, origin.z))

    # create the grid
    start = time.perf_counter()
    grid = ctrigrid_bindings.grid(Nx, Ny, Nz, width, origin)
    grid.add_tri_mesh(mesh_vertices, mesh_indices)
    end = time.perf_counter()
    print("Grid construction finished in {} msecs".format((end - start) * 1000))

    # query closest point
    c = gen_random_points(num_test_points, origin, [Nx * width, Ny * width, Nz * width])

    start = time.perf_counter()
    cp, _ = grid.closest_points(c)   # note that with small number of points, multiple threads is not faster
    end = time.perf_counter()
    print("Grid query finished in {} msecs".format((end - start) * 1000))

    if verbose:
        print('Closest points:')
        print(cp.reshape((-1, 3)))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description='ctrigrid python demo: Load a triangle mesh from n OBJ file, \
            create a grid using ctrigrid and query random points around it.\
            Prints out timings for grid construction & query.')
    parser.add_argument('-i','--infile', required=True, default=None, type=str, 
        help='Input OBJ mesh file.')
    parser.add_argument('-v','--verbose', required=False, default=False, action='store_true', 
        help='Print information about the process.')
    parser.add_argument('-t','--testpoints', required=False, default=16, type=int, 
        help='Number of (randomly generated) tests points')
    parser.add_argument('-n','--N', required=False, default=16, type=int, 
        help='Default dimension of grid (i.e. num of cells). Will be adjusted to \
            lower values for axis with smaller extends')
    parser.add_argument('-p','--padding', required=False, default=0.1, type=float, 
        help='Percentage padding added to the axis aligned bounding box of the input mesh\
                to generate the grid & the test points.')

    args = parser.parse_args()

    main(args.infile, N=args.N, num_test_points=args.testpoints, verbose=args.verbose, padding_perc=args.padding)

