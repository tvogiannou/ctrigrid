#pragma once

// helpers and convenience include for compilation

// platform
// TODO: iOS, MacOS
#if defined(ANDROID) || defined(__ANDROID__)    // check for Android before any other platform since
                                                // in case of cross compilation both platforms are defined
    #define CTRIGRID_PLATFORM_ANDROID 1
#elif defined(_WIN32)
    #define CTRIGRID_PLATFORM_WIN32 1
#elif defined(__linux__)
    #define CTRIGRID_PLATFORM_LINUX 1
#endif


// compiler
#if defined(_MSC_VER)
    #define CTRIGRID_COMPILER_MSVC 1
#elif defined(__clang__) // NOTE: This NEEDS to be checked before __GNUC__ as clang defines both
    #define CTRIGRID_COMPILER_CLANG 1
#elif defined(__GNUC__) || defined(__GNUG__)
    #define CTRIGRID_COMPILER_GCC 1
#endif

// forward header with standard types
#include <cstdint>
#include <float.h>
#ifdef __GLIBCXX__
    using std::size_t; // GNU stdlib++ defines size_t in std (as per standard)
#endif

// helpers
#define CTRIGRID_UNUSED(a) (void)(a);

// build configuration
// TODO: release, optimized, final, etc
#ifdef CTRIGRID_PLATFORM_WIN32
    #ifdef _DEBUG
        #define CTRIGRID_DEBUG_BUILD 1
    #endif
#else // default general case based on NDEBUG
    #ifndef NDEBUG
        #define CTRIGRID_DEBUG_BUILD 1
    #endif
#endif

// Adapter for assert, mainly for more convenient use with the VC debugger 
#ifdef CTRIGRID_DEBUG_BUILD
    #if defined(CTRIGRID_PLATFORM_WIN32)
        #include <CRTDBG.h>
        #include <cstdlib>
        #define CTRIGRID_ASSERT(expr) if(!(expr)) { _CrtDbgBreak(); std::abort(); } // breakpoint for easier debugging
    #else
        #include <cassert>
        #define CTRIGRID_ASSERT(expr) assert(expr)
    #endif
#else
    #define CTRIGRID_ASSERT(expr) CTRIGRID_UNUSED(expr)
#endif