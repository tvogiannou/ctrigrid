## ctrigrid

Closest Triangle (Regular) Grid


<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [ctrigrid](#ctrigrid)
  - [Overview](#overview)
  - [Build Instructions](#build-instructions)
      - [SSE configuration](#sse-configuration)
      - [Generating the tests](#generating-the-tests)
      - [Generating the benchmarks](#generating-the-benchmarks)
  - [Usage](#usage)
    - [Python Bindings](#python-bindings)
  - [Details](#details)
    - [Point Distance Query](#point-distance-query)
    - [Grid Generation](#grid-generation)
    - [Index compression](#index-compression)
  - [FAQ](#faq)

<!-- /code_chunk_output -->


### Overview

ctrigrid is a library implementing method(s) for computing the closest distance between points and an arbitrary [triangle mesh (triangle soup)](https://en.wikipedia.org/wiki/Polygon_soup).
The algorithm employs a [regular grid](https://en.wikipedia.org/wiki/Regular_grid) where each cell stores a list of triangles that are candidates to be the closest ones in the triangle mesh, i.e. all other triangles cannot be closest to any point inside the cell. The core idea behind this approach is that the grid works as a spatial map of the mesh [distance function](https://en.wikipedia.org/wiki/Signed_distance_function), which guarantees the continuity of the function by storing indices to triangles (instead of sampled distance values).  

![](./docs/bunny_demo.gif)


### Build Instructions

The code has been tested with the following compilers
- Visual Studio 2017
- GCC 5.4.0
- Clang 6.0.1

The code has been built & tested using [cmake](https://cmake.org/).
```bash
cd ctrigrid
mkdir build
cd build

cmake ..
cmake --build .
```

##### SSE configuration

The library offers a couple different implementations using [SSE 2 intrinsics](include/ctrigrid/MathOptConfig.h) for some of the low level computations.
See the available options in `include/ctrigrid/MathOptConfig.h` for configuring the build.

##### Generating the tests

The tests in the repo are using [google test](https://github.com/google/googletest) to run.
To generate them pass the following option to the cmake command.
```bash
cd build
cmake -DCTRIGRID_GENERATE_TESTS=1 ..
cmake --build .

# to run the tests
./ctrigrid.AllTests
```

##### Generating the benchmarks

Benchmarks are implemented with [google benchmark](https://github.com/google/benchmark).
To generate them pass the following option to the cmake command.
```bash
cd build
cmake -DCTRIGRID_GENERATE_BENCHMARKS=1 -DCMAKE_BUILD_TYPE=RELEASE ..
cmake --build .

# to run the benchamrks
./ctrigrid.Benchmarks
```

> Benchmarks only work for x64 builds. On Windows, the Win64 generator needs to be passed explicitly to cmake, e.g. for VS 2017 add `-G"Visual Studio 15 Win64"` to the cmake command line, since the default is to use Win32. 

### Usage

```cpp
#include <ctrigrid/ClosestTriUniformGrid.h>

// grid paramters
ctrigrid::ClosestTriUniformGrid::CellKey Nx = ...    // Number of cells along X axis
ctrigrid::ClosestTriUniformGrid::CellKey Ny = ...    // Number of cells along Y axis
ctrigrid::ClosestTriUniformGrid::CellKey Nz = ...    // Number of cells along Z axis
float cellWidth = ...                                // Width of the cell, size is the same along all dimensions
Vector3 origin = ...                                 // Origin of the grid in world space, i.e. cell at index (0, 0, 0) 

ctrigrid::ClosestTriUniformGrid grid;
{
  // setup a builder for creating the grid
  ctrigrid::ClosestTriUniformGrid::Builder builder;

  // initialize a grid builder
  ctrigrid::ClosestTriUniformGrid::Builder::InitInfo info = { Nx, Ny, Nz, origin, cellWidth };
  builder.Init(info);

  // construct from a triangle mesh
  std::vector<float> vertices = ...       // flattened array of floats with the x,y,z coordinates of the mesh vertices
  std::vector<uint32_t> indices = ...     // flattened array of uint32 with the triplets of each triangle vertex
  builder.BeginGridSetup();
  builder.AddTriMesh(vertices, indices);

  // create a grid from the builder
  builder.FinalizeGridSetup(grid);   // data ownership is passed from the builder to the grid, builder is cleared
}

// query a point
ctrigrid::Vector3 testPoint = ...                       // some test point
ctrigrid::Vector3 closestPoint;                         // the 3D position in the surface of the tri mesh that is closest to testPoint 
ctrigrid::ClosestTriUniformGrid::TriKey triKey;  // the index of the mesh triangle where the closestPoint lies on
bool validResult = grid.FindClosestPointOnTris(testPoint, closestPoint, triKey);
```

#### Python Bindings
A subset of the C++ API functionality is exposed in a python module found in the repo. 
To generate and build the python bindings pass the following cmake option 
```bash
cd ctrigrid

# pybind is integrated as a git submodule in the repo so it needs to be updated manually
git submodule update --init

mkdir build
cd build
cmake -DCTRIGRID_GENERATE_PYTHON_BINDINGS=1 ..
```
> It is reccomended to avoid building tests alongside the python bindings since both cmake scripts will try to find the installed python libs and may result in some version conflicts. 

The built python module (as .so/.pyd file) can then be imported in python source.
```python
# example code

import ctrigrid_bindings # this needs to be copied in the local directory

# create the grid
origin = ctrigrid_bindings.vec3(-1.0, -1.0, -1.0)
N = 16
width = 1/8
grid = ctrigrid_bindings.grid(N, N, N, width, origin)
grid.add_tri_mesh(mesh_vertices, mesh_indices)

# query closest point
c = gen_random_points(1024, origin, N, width)
```

### Details

The grid has dimensions `(Nx, Ny, Nz)` and every cell is of the same size, i.e. the dimensions of the cell is `width x width x width`.
The grid origin is the "zero" point in the grid space, i.e. the cell with indices `(0,0,0)`, where the grid expands on each dimension starting from the origin. This is mostly to simplify initialization for meshes aligned at point (0, 0, 0) of the local/mesh space (by setting the grid origin to the "min corner" instead of point zero), since typically the grid should be computed on mesh space.


#### Point Distance Query

Each cell in the grid "contains" a list of candidate nearest triangles in the mesh. Therefore, when querying for the distance of a point to the mesh, the algorithm simply needs to:
- Determine the cell that the point lies into
- Iterate through all the triangles assigned to this cell to find the one with the closest distance.

Optionally, the search for the closest triangle can use bounding sphere checks for early exit (on by default, see `include/ctrigrid/MathOptConfig.h`).

#### Grid Generation  

The grid is generated in 2 major steps:
1) Identify cells that overlap with triangles in the mesh.
2) Iterate for each cell and search for the nearest triangles by expanding the search area to adjacent cells (using an approach fairly similar to [conservative advancement](https://wwwx.cs.unc.edu/~geom/papers/documents/articles/2009/tang09.pdf)), until at least one cell with a triangle from step 1 is visited. All triangles found on all visited cells are added to the currently iterated cell. An extra expansion step is executed in the end to cover for edge cases.

> Currently, the grid generation code has lot of room for improvement as it lacks any optimizations, both in regards to timing performance and search/expansion efficiency.

#### Index compression

To reduce the memory footprint of the grid, the triangle indices for each cell are stored in a compressed stream. The compression scheme is quite straightforward: determine the maximum bits needed to store the triangles and store the cell indices in the bit stream using this number of bits per index (instead of full 32/64 bits representation(s)).
This approach adds some time during the distance query, due to the index decompression cost, but it dramatically reduces the memory requirements of the grid. 

### FAQ
**How to determine the parameters of the grid (Nx/Ny/Nz, cell width, etc)?**  
The parameters of the grid should be such so that the grid fully encloses the mesh **plus the volume that the query points are expected to lie on** (see also next question).
In terms of performance, the smaller the cells the less likely are to be nearing many triangles, so the [distance query](#point-distance-query) should be faster (due to less triangles to iterate on). This however comes at the cost of having to use larger Nx/Ny/Nz values in order to enclose the entire mesh, which results in grid with a larger memory footprint. The trade-off here being between memory and runtime performance, however note that grids large enough can also incur performance costs due to main memory and/or cache access patterns.  
You can also check the [python file test](./python/tests/test_grid_file.py) for a simple heuristic on determining the params for basic usage.

**What happens if the point in the query is not inside the grid?**  
By default, the query returns false and the input is not modified. This can be modified by passing `forceInGrid = false` to the query which will result in approximating the result by finding the closest cell (instead of the enclosing cell). This is not always accurate but for large enough grids it is very close (if not identical) to the correct result.

**When should I enable/disable the SSE implementation?**  
The SSE implementations take 20-40% less computational time on average (single threaded, Core i7), however they incur some loss of accuracy (up to ~3 decimal points). 

**Can I save the grid in disk?**  
There is no built-in serialization for the grid data, even though technically it should not be too involved to implement.
