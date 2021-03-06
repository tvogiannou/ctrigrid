### ctrigrid python tests

This directory contains a small number of python scripts for testing the ctrigrid python bindings.

> The scripts have been tested with python 3.5.

To run the scripts, build the ctrigrid bindings and copy the native module in the directory of the scripts.
The only other dependency is [numpy](https://numpy.org/).

Typically the tests just make sure that execution is successful and simply print the results, but the `test_grid_file.py` is a bit more involved. It takes input options and can load an [OBJ](https://www.fileformat.info/format/wavefrontobj/egff.htm) mesh file, setting up a grid for it with some random points to test queries against. If [PyOpenGL](https://pypi.org/project/PyOpenGL/) and [glut](http://freeglut.sourceforge.net/) are available in the system it can also render the results in 3D. Below is the console output of the script help 

```bash
python3 test_grid_file.py -h

usage: test_grid_file.py [-h] -i INFILE [-v] [-t TESTPOINTS] [-n N]
                         [-p PADDING] [-r]

ctrigrid python demo: Load a triangle mesh from an .OBJ file, create a grid
using ctrigrid and query random points around it. Prints out timings for grid
construction & query.

optional arguments:
  -h, --help            show this help message and exit
  -i INFILE, --infile INFILE
                        Input OBJ mesh file.
  -v, --verbose         Print information about the process.
  -t TESTPOINTS, --testpoints TESTPOINTS
                        Number of (randomly generated) tests points
  -n N, --N N           Default dimension of grid (i.e. num of cells). Will be
                        adjusted to lower values for axis with smaller extends
  -p PADDING, --padding PADDING
                        Percentage padding added to the axis aligned bounding
                        box of the input mesh to generate the grid & the test
                        points.
  -r, --render          Renders the results in new window (if PyOpenGL & glut
                        are available). Use W/S to zoom in/out, A/D to rotate
                        left/right and ESC to close.
```

