#pragma once

#include <ctrigrid/AxisAlignedBoundingBox.h>
#include <ctrigrid/BitStream.h>
#include <ctrigrid/Compiler.h>
#include <ctrigrid/Vector3.h>

#include <vector>
#include <tuple>


namespace ctrigrid
{

// Class for creating, storing and running spatial queries on a set of triangles ("tri soup") via a uniform grid.
// The grid cells are axis aligned have uniform length (boxes) but their number can vary on different axis
class ClosestTriUniformGrid
{
    // non copyable as internal storage to avoid
    ClosestTriUniformGrid(const ClosestTriUniformGrid&) = delete;
    void operator=(const ClosestTriUniformGrid&)=delete;

public:

    using CellIndex = uint32_t;     // type for index in (i, j, k) triplet indexing a grid cell
    using CellKey = uint32_t;       // type to index a grid cell from a single value
    using TriKey = uint32_t;        // type to index a triangle from a single value
    using CellKeyArray = std::vector<TriKey>;
    using TriKeyArray = std::vector<TriKey>;
    
    // utility type for storing triplets of indices
    using CellIndex3 = std::tuple<CellIndex, CellIndex, CellIndex>;
    static CellIndex3 ToIndex3(CellIndex a, CellIndex b, CellIndex c) { return std::make_tuple(a, b, c); }  

    // info stored for every tri
    struct TriInfo
    {
        TriKey idx0, idx1, idx2;
        
        // bounding sphere
        Vector3 sphereCenter;
        float sphereRadius;

        // TODO: add face normal, side normals, etc
    };

    using TriInfoArrayType = std::vector<TriInfo>;
    using TriVertexArrayType = std::vector<Vector3>;


    // indexing & cell methods
    static bool ComputeCellKeyFromIndex(
        CellIndex3 Nxyz, CellIndex3 ijk, CellKey& key);
    static bool ComputeIndexFromCellKey(
        CellIndex3 Nxyz, CellKey key, 
        CellIndex& i, CellIndex& j, CellIndex& k);
    static bool ComputeIndexFromPointGridSpace(
        float cellWidth, const Vector3& point, 
        CellIndex& i, CellIndex& j, CellIndex& k);
    static bool ComputeCellKeyFromPoint(
        CellIndex3 Nxyz, float cellWidth, const AxisAlignedBoundingBox& gridBox,
        const Vector3& point, CellKey& key);


    // utility to build a grid from a vertex & triangle index buffers
    struct Builder
    {
        // data to also be stored in the grid
        TriVertexArrayType      m_vertices;             // array of vertices (grid space)
        TriInfoArrayType        m_tris;                 // array of tris 
        CellIndex3              Nxyz;                   // number of cells along X, Y, Z axis
        float                   m_cellWidth = -1.f;     // width of each cell along all dimensions
        AxisAlignedBoundingBox  m_gridBBoxWorldSpace;   // the bounding box of the entire grid, 
                                                        // i.e. volume of all the cells

        // temp info about tris per cell
        struct TriCellBucket
        {
            TriKeyArray triIndices;    // indices to tris overlapping with this cell
        };
        using CellBucketArrayType = std::vector<TriCellBucket>;
        CellBucketArrayType m_triCells; // cells pointing directly to overlapping tris

        struct InitInfo
        {
            CellIndex    Nx, Ny, Nz;     // number of cells along each axis X/Y/Z
            Vector3      origin;         // origin of the grid in world space
            float        cellWidth;      // width of each cell along all dimensions
        };
        bool Init(const InitInfo& info);
        void Clear();

        // grid construction
        bool BeginGridSetup();

        // vertices and indices are flattened buffers of vertex data, both expected to have sizes that are multiples of 3
        // NOTE: adding multiple meshes is not supported at the moment!
        bool AddTriMesh(const std::vector<float>& vertices, const std::vector<uint32_t>& indices);
        
        // finalize construction and store the result to the input grid
        // mesh data ownership is passed to the grid, rest of builder data is cleared
        bool FinalizeGridSetup(ClosestTriUniformGrid& grid);

        // utils
        void CreateTriInfoFromTriIndices(TriKey idx0, TriKey idx1, TriKey idx2, TriInfo& info);
        TriKey AddTriInfo(const TriInfo& info);
        void AddTriToBucket(CellKey cellKey, TriKey triKey);

        // internal
        bool GetOverlappingTrisOnCell(CellKey key, TriKeyArray& triIndices) const;
        bool ComputeCelBBoxWorldSpace(CellIndex i, CellIndex j, CellIndex k, AxisAlignedBoundingBox& bbox) const;
        bool ComputeCelBBoxGridSpace(CellIndex i, CellIndex j, CellIndex k, AxisAlignedBoundingBox& bbox) const;
    };


    ClosestTriUniformGrid() = default;
    void Clear();

    // accessors
    CellIndex GetNumberCellsXAxis() const { return m_Nx; }
    CellIndex GetNumberCellsYAxis() const { return m_Ny; }
    CellIndex GetNumberCellsZAxis() const { return m_Nz; }
    float GetCellWidth() const { return m_cellWidth; }
    const AxisAlignedBoundingBox& GetGridAABoxWorldSpace() const { return m_gridBBoxWorldSpace; }
    const TriInfoArrayType& GetTriangles() const { return m_tris; }
    const TriVertexArrayType& GetVertices() const { return m_vertices; }


    // queries

    // returns the full list of triangles that are "closest" to the cell with the input key
    // closest in this context means that any other triangle cannot be closer to the cell than one (or more) in the list
    // in practice, this is the list of triangles that will be evaluated during a distance query
    // triIndices refer to the grid triangle list 
    bool GetClosestTrisOnCell(CellKey key, TriKeyArray& triIndices) const;
    bool GetTriVerticesWorldSpace(TriKey triKey, Vector3& v0, Vector3& v1, Vector3& v2) const;

    // find the closest point to p and the respective tri key
    // forceInGrid can be set to only look perform the query if p lies in the grid, if set to false then
    // the query is using the closest cell to p instead which is just an approximation and 
    // may lead to errors in edge cases
    bool FindClosestPointOnTris(
        const Vector3& p, 
        Vector3& closestPoint, TriKey& triKey, bool forceInGrid = true) const;


    // internal
    
    // an estimate of the used memory by the grid
    struct MemoryStats
    {
        uint64_t verticesAllocMem;
        uint64_t trisAllocMem;
        uint64_t cellsAllocMem;
        uint64_t cellIndicesAllocMem;
    };
    MemoryStats ComputeMemFootprint() const;

private:

    // vertex data buffers
    TriVertexArrayType      m_vertices; // array of vertices (grid space)
    TriInfoArrayType        m_tris;     // array of tris 

    // size & position data
    CellIndex   m_Nx = 0u;              // number of cells along X axis
    CellIndex   m_Ny = 0u;              // number of cells along Y axis
    CellIndex   m_Nz = 0u;              // number of cells along Z axis
    float       m_cellWidth = -1.f;     // width of each cell along all dimensions
    AxisAlignedBoundingBox  m_gridBBoxWorldSpace;   // the bounding box of the entire grid, i.e. volume of all the cells

    // compressed tri indices stored in the grid cells for queries
    uint8_t                             m_indexBitWidth;
    BitStreamBuffer                     m_indexBitStream;
    BitStreamBuffer::BufferIndex        m_lastBitPos;
    BitStreamBuffer::BufferIndexArray   m_indexCells;

    friend class ClosestTriUniformGrid::Builder;
};

}