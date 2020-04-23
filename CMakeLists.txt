cmake_minimum_required (VERSION 3.5.1)
project(ctrigrid)


option (CTRIGRID_GENERATE_TESTS "Generate unit tests" OFF)
option (CTRIGRID_GENERATE_BENCHMARKS "Generate benchmarks" OFF)
option (CTRIGRID_GENERATE_PYTHON_BINDINGS "Generate target for python bindings" OFF)


set(CTRIGRID_EXTERNAL_DEPENDENCIES_LOCATION "${PROJECT_SOURCE_DIR}/external")

if(CTRIGRID_GENERATE_BENCHMARKS)
    message("Adding Google Benchmark...")
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(	"${CTRIGRID_EXTERNAL_DEPENDENCIES_LOCATION}/benchmark-1.5.0" 
                        ${CMAKE_CURRENT_BINARY_DIR}/benchmark-build
                        EXCLUDE_FROM_ALL)
endif(CTRIGRID_GENERATE_BENCHMARKS)

# compiler settings
set(CMAKE_CXX_STANDARD 11)
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /GR- /WX /Zc:wchar_t-")
endif(MSVC)
if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -fno-rtti")

    if(CTRIGRID_GENERATE_PYTHON_BINDINGS)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti -ldl -lpthread -fpic")   # rtti for pybind11
    endif()
endif(UNIX)


set(CTRIGRID_SOURCE_FILES 
    ${CMAKE_CURRENT_LIST_DIR}/src/AxisAlignedBoundingBox.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/BitStream.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ClosestTriUniformGrid.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ClosestTriUniformGrid_Builder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/TriQueries.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Vector3.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Vector4.cpp
    ${CMAKE_CURRENT_LIST_DIR}/include/ctrigrid/AxisAlignedBoundingBox.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ctrigrid/BitStream.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ctrigrid/ClosestTriUniformGrid.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ctrigrid/Compiler.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ctrigrid/MathOptConfig.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ctrigrid/TriQueries.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ctrigrid/Vector3.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ctrigrid/Vector4.h)
set(CTRIGRID_PUBLIC_HEADERS_DIR ${CMAKE_CURRENT_LIST_DIR}/include)

# main target
add_library(ctrigrid STATIC ${CTRIGRID_SOURCE_FILES})

target_include_directories(ctrigrid PUBLIC "${CTRIGRID_PUBLIC_HEADERS_DIR}")


if(CTRIGRID_GENERATE_TESTS)
    message("Adding Google Test...")
    # add the google test libs in the project
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(GOOGLETEST_VERSION 1.10.0)
    add_subdirectory("${CTRIGRID_EXTERNAL_DEPENDENCIES_LOCATION}/googletest-release-1.10.0" 
                 ${CMAKE_CURRENT_BINARY_DIR}/gtest-build
                 EXCLUDE_FROM_ALL)
    
    file(GLOB CTRIGRID_TEST_FILES "${CMAKE_CURRENT_LIST_DIR}/tests/*.cpp" "tests/*.h")
    add_executable(ctrigrid.AllTests ${CTRIGRID_TEST_FILES})
    target_link_libraries(ctrigrid.AllTests ctrigrid gtest_main)
endif(CTRIGRID_GENERATE_TESTS)

if(CTRIGRID_GENERATE_BENCHMARKS)
    file(GLOB CTRIGRID_BENCHMARK_FILES "${CMAKE_CURRENT_LIST_DIR}/benchmarks/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/benchmarks/*.h")
    add_executable(ctrigrid.Benchmarks ${CTRIGRID_BENCHMARK_FILES})
    target_link_libraries(ctrigrid.Benchmarks ctrigrid benchmark benchmark_main)
endif(CTRIGRID_GENERATE_BENCHMARKS)

if (CTRIGRID_GENERATE_PYTHON_BINDINGS)
    add_subdirectory(${CTRIGRID_EXTERNAL_DEPENDENCIES_LOCATION}/pybind11  ${CMAKE_CURRENT_BINARY_DIR}/pybind11-build)

    # create pybind11 module
    pybind11_add_module(ctrigrid_bindings 
                        ${CMAKE_CURRENT_LIST_DIR}/python/module.cpp
                        ${CMAKE_CURRENT_LIST_DIR}/python/queries_bindings.cpp
                        ${CMAKE_CURRENT_LIST_DIR}/python/mesh_bindings.cpp
                        ${CMAKE_CURRENT_LIST_DIR}/python/module.h)
    target_include_directories(ctrigrid_bindings PRIVATE ${CTRIGRID_EXTERNAL_DEPENDENCIES_LOCATION}/tiny_obj_loader)
    target_link_libraries(ctrigrid_bindings PRIVATE ctrigrid)
endif(CTRIGRID_GENERATE_PYTHON_BINDINGS)