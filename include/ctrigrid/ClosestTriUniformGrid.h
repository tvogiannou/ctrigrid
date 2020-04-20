#pragma once

#include <ctrigrid/Compiler.h>
#include <ctrigrid/Vector3.h>
#include <ctrigrid/AxisAlignedBoundingBox.h>
#include <ctrigrid/BitStream.h>

#include <vector>


namespace ctrigrid
{

// Class for creating, storing and running spatial queries on a set of triangles ("tri soup") via a uniform grid.
// The grid cells are axis aligned have uniform length (boxes) but their number can vary on different axis
class ClosestTriUniformGrid
{
    // non copyable
    ClosestTriUniformGrid(const ClosestTriUniformGrid&) = delete;
    void operator=(const ClosestTriUniformGrid&)=delete;

public:

    using MapIndexType = uint32_t;
    using MapCellKeyType = uint32_t;
    using MapTriKeyType = uint32_t;
    using MapCellKeyArrayType = std::vector<MapTriKeyType>;
    using MapTriKeyArrayType = std::vector<MapTriKeyType>;

    ClosestTriUniformGrid() = default;

    // indexing & cell methods
    bool ComputeCellKeyFromIndex(MapIndexType i, MapIndexType j, MapIndexType k, MapCellKeyType& key) const;
    bool ComputeIndexFromCellKey(MapCellKeyType key, MapIndexType& i, MapIndexType& j, MapIndexType& k) const;
    bool ComputeCellKeyFromPoint(const Vector3& point, MapCellKeyType& key) const;
    bool ComputeIndexFromPointGridSpace(const Vector3& point, MapIndexType& i, MapIndexType& j, MapIndexType& k) const;
    bool ComputeCelBBoxWorldSpace(MapIndexType i, MapIndexType j, MapIndexType k, AxisAlignedBoundingBox& bbox) const;
    bool ComputeCelBBoxGridSpace(MapIndexType i, MapIndexType j, MapIndexType k, AxisAlignedBoundingBox& bbox) const;

    // accessors
    bool GetClosestTrisOnCell(MapCellKeyType key, MapTriKeyArrayType& triIndices) const;
    bool GetTriVerticesWorldSpace(MapTriKeyType triKey, Vector3& v0, Vector3& v1, Vector3& v2) const;
    MapIndexType GetNumberCellsXAxis() const { return m_Nx; }
    MapIndexType GetNumberCellsYAxis() const { return m_Ny; }
    MapIndexType GetNumberCellsZAxis() const { return m_Nz; }
    float GetCellWidth() const { return m_cellWidth; }
    const AxisAlignedBoundingBox& GetGridAABoxWorldSpace() const { return m_gridBBoxWorldSpace; }

    // object init/clear
    struct InitInfo
    {
        MapIndexType    Nx, Ny, Nz;     // number of cells along each axis X/Y/Z
        Vector3         origin;         // origin of the grid in world space
        float           cellWidth;      // width of each cell along all dimensions
    };
    bool Init(const InitInfo& info);
    bool Clear();

    // grid construction
    // vertices and indices are flattened buffers of vertex data, both expected to have sizes that are multiples of 3
    // NOTE: adding multiple meshes is not supported at the moment!
    bool BeginGridSetup();
    bool AddTriMesh(const std::vector<float>& vertices, const std::vector<uint32_t>& indices);
    bool FinalizeGridSetup();

    // queries
    // find the closest point to p and the respective tri key
    // forceInGrid can be set to only look perform the query if p lies in the grid, if set to false then
    // the query is using the closest cell to p instead which is just an approximation and may lead to errors
    // in cases
    bool FindClosestPointOnTris(const Vector3& p, Vector3& closestPoint, MapTriKeyType& triKey, bool forceInGrid = true) const;
    // TODO: query buffer of points
    
    // TODO: update
    //void ApplyTranslation(const Vector3& T);
    //void ApplyTtransform(const Matrix4& X);

    // internal
    struct MemoryStats
    {
        uint64_t verticesAllocMem;
        uint64_t trisAllocMem;
        uint64_t cellsAllocMem;
        uint64_t cellIndicesAllocMem;
    };
    // an estimate of the used memory by the grid
    MemoryStats ComputeMemFootprint() const;

    // internal
    bool GetOverlappingTrisOnCell(MapCellKeyType key, MapTriKeyArrayType& triIndices) const;

private:

    // info stored for every tri
    struct TriInfo
    {
        MapTriKeyType idx0, idx1, idx2;
        
        // bounding sphere
        Vector3 sphereCenter;
        float sphereRadius;

        // TODO: add face normal, side normals, etc
    };
    void CreateTriInfoFromTriIndices(MapTriKeyType idx0, MapTriKeyType idx1, MapTriKeyType idx2, TriInfo& info);
    MapTriKeyType AddTriInfo(const TriInfo& info);

    // vertex data buffers
    // TODO: optimize storage using re-sizable buffers on creation & fixed buffers for saving/loading/queries
    using TriInfoArrayType = std::vector<TriInfo>;
    using TriVertexArrayType = std::vector<Vector3>;
    TriVertexArrayType m_vertices;  // array of vertices (grid space)
    TriInfoArrayType m_tris;        // array of tris 

    // size & position data
    MapIndexType            m_Nx = 0u;              // number of cells along X axis
    MapIndexType            m_Ny = 0u;              // number of cells along Y axis
    MapIndexType            m_Nz = 0u;              // number of cells along Z axis
    float                   m_cellWidth = -1.f;     // width of each cell along all dimensions
    AxisAlignedBoundingBox  m_gridBBoxWorldSpace;   // the bounding box of the entire grid, i.e. volume of all the cells

    // compressed tri indices stored in the grid cells for queries
    uint8_t m_indexBitWidth;
    BitStreamBuffer::BufferIndexType m_lastBitPos;
    BitStreamBuffer m_indexBitStream;
    std::vector<BitStreamBuffer::BufferIndexType> m_indexCells;


    // temporary info stored for every cell during creation
    struct TriCellBucket
    {
        MapTriKeyArrayType triIndices;    // indices to tris overlapping with this cell
    };
    void AddTriToBucket(MapCellKeyType cellKey, MapTriKeyType triKey);
    using CellBucketArrayType = std::vector<TriCellBucket>;
    CellBucketArrayType m_triCells; // cells pointing directly to overlapping tris
};

}