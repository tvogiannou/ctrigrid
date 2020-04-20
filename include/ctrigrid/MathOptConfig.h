#pragma once

#include <ctrigrid/Compiler.h>

// configure settings for optimized implementations

// enable SSE instructions on tri-point distance query 
#define CTRIGRID_TRIPOINTDISTQUERY_SSE

// enable bounds check while traversing the triangles in the grid cell 
#define CTRIGRID_GRID_BOUNDS_CHECK

// enable SSE instructions on bounds computations while traversing the triangles in the grid cell 
// unused if CTRIGRID_GRID_BOUNDS_CHECK is not defined
#define CTRIGRID_GRID_BOUNDS_CHECK_SSE


// include sse headers if any sse implementation is enabled
#if defined(CTRIGRID_TRIPOINTDISTQUERY_SSE) || defined(CTRIGRID_GRID_BOUNDS_CHECK_SSE)
#ifdef CTRIGRID_COMPILER_MSVC
    #include <intrin.h>
#elif defined(CTRIGRID_COMPILER_GCC) || defined(CTRIGRID_COMPILER_CLANG)
    #include <xmmintrin.h>
    #include <emmintrin.h>
#endif
#endif
