
#pragma  once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <tuple>
#include <string>

#include <ctrigrid/Compiler.h>
#include <ctrigrid/TriQueries.h>
#include <ctrigrid/ClosestTriUniformGrid.h>



struct CTRIGRID_Vector3_wrapper
{
    CTRIGRID_Vector3_wrapper(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    ctrigrid::Vector3 ToVector3() const { return ctrigrid::Vector3(x, y, z); }


    float x, y, z;
};

struct CTRIGRID_UniformGrid_wrapper
{
    CTRIGRID_UniformGrid_wrapper(uint32_t Nx, uint32_t Ny, uint32_t Nz, float cellWidth,
        const CTRIGRID_Vector3_wrapper& origin);

    void Begin();
    void Finalize();
    void AddTri(
        const CTRIGRID_Vector3_wrapper& v0,
        const CTRIGRID_Vector3_wrapper& v1,
        const CTRIGRID_Vector3_wrapper& v2);

    void AddTris(
        pybind11::array_t<float, pybind11::array::c_style | pybind11::array::forcecast> vertices,
        pybind11::array_t<uint32_t> indices);

    std::tuple<CTRIGRID_Vector3_wrapper, size_t> FindClosestPointOnTris(const CTRIGRID_Vector3_wrapper& p) const;
    std::tuple<pybind11::array_t<float>, pybind11::array_t<uint32_t>> 
        FindAllClosestPointsOnTris(pybind11::array_t<float> points, bool forceInGrid = false) const;

    ctrigrid::ClosestTriUniformGrid::Builder::InitInfo info;
    ctrigrid::ClosestTriUniformGrid grid;

    // non-copyable
    CTRIGRID_UniformGrid_wrapper(const CTRIGRID_UniformGrid_wrapper&) = delete;
    CTRIGRID_UniformGrid_wrapper& operator=(const CTRIGRID_UniformGrid_wrapper&) = delete;
};

// wrapper to ClosestDistanceQuery::ClosestPointOnTri
CTRIGRID_Vector3_wrapper CTRIGRID_ComputeClosestPointOnTri(
    const CTRIGRID_Vector3_wrapper& v0,
    const CTRIGRID_Vector3_wrapper& v1,
    const CTRIGRID_Vector3_wrapper& v2,
    const CTRIGRID_Vector3_wrapper& p);

// helpers to compute the normals of a tri mesh defined by two python vertex & index buffers
pybind11::array_t<float> CTRIGRID_ComputeVertexNormals(
    pybind11::array_t<float, pybind11::array::c_style | pybind11::array::forcecast> vertices,
    pybind11::array_t<uint32_t> indices);
pybind11::array_t<float> CTRIGRID_ComputeTriNormals(
    pybind11::array_t<float, pybind11::array::c_style | pybind11::array::forcecast> vertices,
    pybind11::array_t<uint32_t> indices);

// helper to find the unique edges in a tri mesh
pybind11::array_t<uint32_t> CTRIGRID_GetUniqueEdges(pybind11::array_t<uint32_t> indices);

// helper for importing tri mesh from OBJ file
std::tuple<pybind11::array_t<float>, pybind11::array_t<uint32_t>> CTRIGRID_LoadObjFile(const std::string& filename);

// helpers savinga tri mesh to a file using the OBJ format
void CTRIGRID_SaveObjFile(const std::string& filename,
    pybind11::array_t<float, pybind11::array::c_style | pybind11::array::forcecast> vertices,
    pybind11::array_t<uint32_t> indices);