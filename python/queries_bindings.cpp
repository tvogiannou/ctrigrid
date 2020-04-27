
#include "module.h"

#include <ctrigrid/TriQueries.h>

#include <algorithm>
#include <thread>
#include <vector>
#include <functional>


CTRIGRID_UniformGrid_wrapper::CTRIGRID_UniformGrid_wrapper(uint32_t Nx, uint32_t Ny, uint32_t Nz, float cellWidth,
    const CTRIGRID_Vector3_wrapper& origin)
{
    info.Nx = Nx;
    info.Ny = Ny;
    info.Nz = Nz;
    info.cellWidth = cellWidth;
    info.origin = origin.ToVector3();
}

void CTRIGRID_UniformGrid_wrapper::Begin() {  }
void CTRIGRID_UniformGrid_wrapper::Finalize() {  }


void CTRIGRID_UniformGrid_wrapper::AddTris(
    pybind11::array_t<float, pybind11::array::c_style | pybind11::array::forcecast> vertices,
    pybind11::array_t<uint32_t> indices)
{
    ctrigrid::ClosestTriUniformGrid::Builder builder;
    if(!builder.Init(info))
        throw std::runtime_error("Grid builder init failed: internal error");
    
    pybind11::buffer_info vertexBufferInfo = vertices.request();
    pybind11::buffer_info indexBufferInfo = indices.request();

    if (vertexBufferInfo.ndim != 1u || indexBufferInfo.ndim != 1u)
        throw std::runtime_error("Number of dimensions must be one");

    const float* vtx = (const float*)vertexBufferInfo.ptr;
    const uint32_t* idx = (const uint32_t*)indexBufferInfo.ptr;

    const size_t numVertices = vertexBufferInfo.shape[0];
    const size_t numIndices = indexBufferInfo.shape[0];

    std::vector<float> v;
    std::vector<uint32_t> i;
    v.resize(numVertices);
    i.resize(numIndices);
    std::memcpy(v.data(), vtx, numVertices * sizeof(float));
    std::memcpy(i.data(), idx, numIndices * sizeof(uint32_t));

    if (!builder.BeginGridSetup()) 
        throw std::runtime_error("Grid builder begin failed: internal error");
    if (!builder.AddTriMesh(v, i))
        throw std::runtime_error("Grid builder AddTriMesh failed: internal error");
    if (!builder.FinalizeGridSetup(grid)) 
        throw std::runtime_error("Grid builder finalize failed: internal error");
}

std::tuple<CTRIGRID_Vector3_wrapper, size_t> 
CTRIGRID_UniformGrid_wrapper::FindClosestPointOnTris(const CTRIGRID_Vector3_wrapper& p) const
{
    const ctrigrid::Vector3 _p = p.ToVector3();

    ctrigrid::Vector3 _closestPoint;
    ctrigrid::ClosestTriUniformGrid::MapTriKeyType triKey;
    if(!grid.FindClosestPointOnTris(_p, _closestPoint, triKey))
        throw std::runtime_error("Grid FindClosestPointOnTris failed: internal error");

    CTRIGRID_Vector3_wrapper closestPoint(0.f, 0.f, 0.f);
    size_t triIndex;
    
    closestPoint.x = _closestPoint.x;
    closestPoint.y = _closestPoint.y;
    closestPoint.z = _closestPoint.z;

    triIndex = triKey;

    return std::make_tuple(closestPoint, triIndex);
}

std::tuple<pybind11::array_t<float>, pybind11::array_t<uint32_t>>
CTRIGRID_UniformGrid_wrapper::FindAllClosestPointsOnTris(pybind11::array_t<float> points,
                                bool forceInGrid /*= false*/) const
{
    pybind11::buffer_info pointsBufferInfo = points.request();
    if (pointsBufferInfo.ndim != 1u)
        throw std::runtime_error("Number of dimensions must be one");

    // allocate return buffers
    auto closestPointsBuffer = pybind11::array_t<float>(pointsBufferInfo.size);
    auto closestTrisBuffer = pybind11::array_t<uint32_t>(pointsBufferInfo.size / 3u);
    pybind11::buffer_info closestPointsBufferInfo = closestPointsBuffer.request();
    pybind11::buffer_info closestTrisBufferInfo = closestTrisBuffer.request();

    const float* pts = (const float*)pointsBufferInfo.ptr;
    float* closestPts = (float*)closestPointsBufferInfo.ptr;
    uint32_t* closestTris = (uint32_t*)closestTrisBufferInfo.ptr;

    // check all points
    {
        for (size_t i = 0; i < (size_t)pointsBufferInfo.shape[0]; i += 3u)
        {
            const ctrigrid::Vector3 p(pts[i], pts[i + 1], pts[i + 2]);

            ctrigrid::Vector3 _closestPoint;
            ctrigrid::ClosestTriUniformGrid::MapTriKeyType triKey;
            //bool forceInGrid = false;
            if (!grid.FindClosestPointOnTris(p, _closestPoint, triKey, forceInGrid))
                throw std::runtime_error("Grid FindClosestPointOnTris failed: internal error");

            closestTris[i / 3u] = triKey;
            closestPts[i] = _closestPoint.x;
            closestPts[i + 1] = _closestPoint.y;
            closestPts[i + 2] = _closestPoint.z;
        }
    }

    return std::make_tuple(closestPointsBuffer, closestTrisBuffer);
}

// wrapper to ClosestDistanceQuery::ClosestPointOnTri
CTRIGRID_Vector3_wrapper CTRIGRID_ComputeClosestPointOnTri(
    const CTRIGRID_Vector3_wrapper& v0,
    const CTRIGRID_Vector3_wrapper& v1,
    const CTRIGRID_Vector3_wrapper& v2,
    const CTRIGRID_Vector3_wrapper& p)
{
    const ctrigrid::Vector3 _v0 = v0.ToVector3();
    const ctrigrid::Vector3 _v1 = v1.ToVector3();
    const ctrigrid::Vector3 _v2 = v2.ToVector3();
    const ctrigrid::Vector3 _p = p.ToVector3();

    CTRIGRID_Vector3_wrapper closestPoint(0.f, 0.f, 0.f);
    ctrigrid::Vector3 _closestPoint;
    if (ctrigrid::ClosestDistanceQuery::ClosestPointOnTri(_p, _v0, _v1, _v2, _closestPoint))
    {
        closestPoint.x = _closestPoint.x;
        closestPoint.y = _closestPoint.y;
        closestPoint.z = _closestPoint.z;
    }
    else
        throw std::runtime_error("Failed to compute closest point: algorithm error");

    return closestPoint;
}
