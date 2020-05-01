
#include <ctrigrid/ClosestTriUniformGrid.h>

#include <ctrigrid/Compiler.h>
#include <ctrigrid/TriQueries.h>


namespace ctrigrid
{

static void 
CreateTriInfoFromTriIndices(
    const ClosestTriUniformGrid::Builder& builder,
    ClosestTriUniformGrid::TriKey idx0, 
    ClosestTriUniformGrid::TriKey idx1, 
    ClosestTriUniformGrid::TriKey idx2, 
    ClosestTriUniformGrid::TriInfo& info)
{
    info.idx0 = idx0;
    info.idx1 = idx1;
    info.idx2 = idx2;

    // compute bounding sphere
    {
        CTRIGRID_ASSERT(info.idx0 < builder.vertices.size());
        CTRIGRID_ASSERT(info.idx1 < builder.vertices.size());
        CTRIGRID_ASSERT(info.idx2 < builder.vertices.size());

        const Vector3& v0 = builder.vertices[info.idx0];
        const Vector3& v1 = builder.vertices[info.idx1];
        const Vector3& v2 = builder.vertices[info.idx2];

        // TODO: improve this by computing smaller sphere
        // centroid for center
        info.sphereCenter = v0;
        info.sphereCenter.Add(v1);
        info.sphereCenter.Add(v2);
        info.sphereCenter.Mul(1.f / 3.f);

        Vector3 d = v0;
        d.Sub(info.sphereCenter);
        const float d0 = d.Dot(d);

        d = v1;
        d.Sub(info.sphereCenter);
        const float d1 = d.Dot(d);

        d = v2;
        d.Sub(info.sphereCenter);
        const float d2 = d.Dot(d);

        info.sphereRadius = sqrtf(std::max({ d0, d1, d2 }));
    }
}

static ClosestTriUniformGrid::CellKey 
AddTriInfo(
    ClosestTriUniformGrid::Builder& builder,
    const ClosestTriUniformGrid::TriInfo& info)
{
    builder.tris.emplace_back(info);

    return (ClosestTriUniformGrid::CellKey)(builder.tris.size() - 1u);
}

static void 
AddTriToBucket(
    ClosestTriUniformGrid::Builder& builder,
    ClosestTriUniformGrid::CellKey cellKey, 
    ClosestTriUniformGrid::TriKey triKey)
{
    ClosestTriUniformGrid::Builder::TriCellBucket& bucket = 
                                builder.triCells.at(cellKey);

    // TODO: keep sorted to optimize search
    if (std::find(bucket.triIndices.begin(), bucket.triIndices.end(), triKey) == bucket.triIndices.end())
        bucket.triIndices.push_back(triKey);
}

// Helper representing the border of a region ("shell") in the grid
// TODO: change this so it keeps tracks the cells that have been checked while expanding
struct GridBorderRegionHelper
{
    GridBorderRegionHelper(
        ClosestTriUniformGrid::CellIndex _Nx,
        ClosestTriUniformGrid::CellIndex _Ny,
        ClosestTriUniformGrid::CellIndex _Nz) 
        : Nx(_Nx), Ny(_Ny), Nz(_Nz)
    {}

    const ClosestTriUniformGrid::CellIndex Nx;
    const ClosestTriUniformGrid::CellIndex Ny;
    const ClosestTriUniformGrid::CellIndex Nz;

    ClosestTriUniformGrid::CellIndex iMin, iMax;
    ClosestTriUniformGrid::CellIndex jMin, jMax;
    ClosestTriUniformGrid::CellIndex kMin, kMax;

    void Reset(
        ClosestTriUniformGrid::CellIndex i,
        ClosestTriUniformGrid::CellIndex j,
        ClosestTriUniformGrid::CellIndex k)
    {
        iMin = iMax = i;
        jMin = jMax = j;
        kMin = kMax = k;
    }

    bool Expand()
    {
        bool expanded = false;

        if (iMin > 0u) { --iMin; expanded = true; }
        if (iMax + 1u < Nx ) { ++iMax; expanded = true; }

        if (jMin > 0u) { --jMin; expanded = true; }
        if (jMax + 1u < Ny) { ++jMax;expanded = true; }

        if (kMin > 0u) { --kMin; expanded = true; }
        if (kMax + 1u < Nz) { ++kMax; expanded = true; }

        return expanded;
    }

    bool hasFinished()
    {
        return  iMin == 0u && iMax + 1u == Nx &&
                jMin == 0u && jMax + 1u == Ny &&
                kMin == 0u && kMax + 1u == Nz;
    }

    // gets the keys for the cells that lie in the border of this region
    bool GetCellKeysFromRegion(
        const ClosestTriUniformGrid::Builder& builder,
        ClosestTriUniformGrid::CellKeyArray& cellRegionKeys)
    {
        // special case
        if (iMin == iMax && jMin == jMax && kMin == kMax)
        {
            ClosestTriUniformGrid::CellKey key;
            if (!ClosestTriUniformGrid::ComputeCellKeyFromIndex(
                builder.Nxyz, ClosestTriUniformGrid::ToIndex3(iMin, jMin, kMin), key))
                return false;
            cellRegionKeys.push_back(key);

            return true;
        }

        // +-z sides of border
        for (ClosestTriUniformGrid::CellIndex i = iMin; i <= iMax; ++i)
        {
            for (ClosestTriUniformGrid::CellIndex j = jMin; j <= jMax; ++j)
            {
                ClosestTriUniformGrid::CellKey key;
                if (!ClosestTriUniformGrid::ComputeCellKeyFromIndex(
                    builder.Nxyz, ClosestTriUniformGrid::ToIndex3(i, j, kMin), key)) 
                    return false;
                cellRegionKeys.push_back(key);

                if (kMin != kMax)
                {
	                if (!ClosestTriUniformGrid::ComputeCellKeyFromIndex(
                        builder.Nxyz, ClosestTriUniformGrid::ToIndex3(i, j, kMax), key))
	                    return false;
	                cellRegionKeys.push_back(key);
                }
            }
        }

        for (ClosestTriUniformGrid::CellIndex k = kMin + 1u; k < kMax; ++k)
        {
            // =-y sides
            for (ClosestTriUniformGrid::CellIndex i = iMin; i <= iMax; ++i)
            {
                ClosestTriUniformGrid::CellKey key;
                if (!ClosestTriUniformGrid::ComputeCellKeyFromIndex(
                    builder.Nxyz, ClosestTriUniformGrid::ToIndex3(i, jMin, k), key))
                    return false;
                cellRegionKeys.push_back(key);

                if (jMin != jMax)
                {
                    if (!ClosestTriUniformGrid::ComputeCellKeyFromIndex(
                        builder.Nxyz, ClosestTriUniformGrid::ToIndex3(i, jMax, k), key))
                        return false;
                    cellRegionKeys.push_back(key);
                }
            }

            // +-x sides
            for (ClosestTriUniformGrid::CellIndex j = jMin; j <= jMax; ++j)
            {
                ClosestTriUniformGrid::CellKey key;
                if (!ClosestTriUniformGrid::ComputeCellKeyFromIndex(
                    builder.Nxyz, ClosestTriUniformGrid::ToIndex3(iMin, j, k), key))
                    return false;
                cellRegionKeys.push_back(key);

                if (iMin != iMax)
                {
                    if (!ClosestTriUniformGrid::ComputeCellKeyFromIndex(
                        builder.Nxyz, ClosestTriUniformGrid::ToIndex3(iMax, j, k), key))
                        return false;
                    cellRegionKeys.push_back(key);
                }
            }
        }

        return true;
    }
};

ClosestTriUniformGrid::Builder::BuilderStatus 
ClosestTriUniformGrid::Builder::ComputeCelBBoxWorldSpace(
    CellIndex i, CellIndex j, CellIndex k, AxisAlignedBoundingBox& bbox) const
{
    if (ComputeCelBBoxGridSpace(i, j, k, bbox) != BuilderStatus::eBUILDER_STATUS_SUCCESS)
        return BuilderStatus::eBUILDER_STATUS_INVALID_INDEX;

    // add grid origin
    bbox.min.Add(gridBox.min);
    bbox.max.Add(gridBox.min);

    return BuilderStatus::eBUILDER_STATUS_SUCCESS;
}

ClosestTriUniformGrid::Builder::BuilderStatus 
ClosestTriUniformGrid::Builder::ComputeCelBBoxGridSpace(
    CellIndex i, CellIndex j, CellIndex k, AxisAlignedBoundingBox& bbox) const
{
    CellIndex Nx, Ny, Nz;
    std::tie(Nx, Ny, Nz) = Nxyz;

    if (i >= Nx || j >= Ny || k >= Nz)
        return BuilderStatus::eBUILDER_STATUS_INVALID_INDEX;

    // compute start and end points of cell bbox
    Vector3 p0(((float)i) * cellWidth, ((float)j) * cellWidth, ((float)k) * cellWidth);
    Vector3 p1 = p0;
    p1.Add(cellWidth);

    bbox.Reset();
    bbox.AddPoint(p0);
    bbox.AddPoint(p1);

    return BuilderStatus::eBUILDER_STATUS_SUCCESS;
}

ClosestTriUniformGrid::Builder::BuilderStatus 
ClosestTriUniformGrid::Builder::Init(const InitInfo& info)
{
    if (info.Nx == 0u || info.Ny == 0u || info.Nz == 0u)
        return BuilderStatus::eBUILDER_STATUS_INVALID_PARAM_N;
    if (info.cellWidth <= 0.f)
        return BuilderStatus::eBUILDER_STATUS_INVALID_PARAM_WIDTH;

    Nxyz = ClosestTriUniformGrid::ToIndex3(info.Nx, info.Ny, info.Nz);

    cellWidth = info.cellWidth;

    Vector3 maxExtend(((float)info.Nx) * cellWidth, 
                      ((float)info.Ny) * cellWidth, 
                      ((float)info.Nz) * cellWidth);
    maxExtend.Add(info.origin);
    
    gridBox.Reset();
    gridBox.AddPoint(info.origin);
    gridBox.AddPoint(maxExtend);

    return BuilderStatus::eBUILDER_STATUS_SUCCESS;
}

void 
ClosestTriUniformGrid::Builder::Clear()
{
    Nxyz = ClosestTriUniformGrid::ToIndex3(0u, 0u, 0u);
    cellWidth = -1;
    gridBox.Reset();

    tris.clear();
    vertices.clear();
    triCells.clear();
}

ClosestTriUniformGrid::Builder::BuilderStatus 
ClosestTriUniformGrid::Builder::BeginGridSetup()
{
    if (!vertices.empty() || !tris.empty() || !triCells.empty())
        return BuilderStatus::eBUILDER_STATUS_NONEMPTY_MESH;

    CellIndex Nx, Ny, Nz;
    std::tie(Nx, Ny, Nz) = Nxyz;
    if (Nx == 0u || Ny == 0u || Nz == 0u)
        return BuilderStatus::eBUILDER_STATUS_INVALID_PARAM_N;

    triCells.resize(Nx * Ny * Nz);

    return BuilderStatus::eBUILDER_STATUS_SUCCESS;
}

ClosestTriUniformGrid::Builder::BuilderStatus  
ClosestTriUniformGrid::Builder::FinalizeGridSetup(ClosestTriUniformGrid& grid)
{
    if (tris.empty() || triCells.empty())
        return BuilderStatus::eBUILDER_STATUS_NONINITIALIZED;

    grid.Clear();
    std::tie(grid.m_Nx, grid.m_Ny, grid.m_Nz) = Nxyz;

    if (grid.m_Nx == 0u || grid.m_Ny == 0u || grid.m_Nz == 0u)
        return BuilderStatus::eBUILDER_STATUS_INVALID_PARAM_N;
    if (cellWidth <= 0.f)
        return BuilderStatus::eBUILDER_STATUS_INVALID_PARAM_WIDTH;

    // figure out the width required for the number of tris we got
    {
        size_t maxTriIndex = tris.size();
        grid.m_indexBitWidth = 1;
        while (maxTriIndex >>= 1u) ++grid.m_indexBitWidth;
    }
    
    // setup the bit stream for the indices
    grid.m_indexBitStream.Clear();
    BitStreamWriter writer(grid.m_indexBitStream, grid.m_indexBitWidth);
    writer.Begin(tris.size() + sizeof(uint64_t) + 1u);    // rough estimate for initialization
    grid.m_indexCells.clear();
    grid.m_indexCells.resize(grid.m_Nx * grid.m_Ny * grid.m_Nz);
    grid.m_indexCells.shrink_to_fit();

    // loop through all the cells and find the nearest ones with tris
    for (ClosestTriUniformGrid::CellIndex k = 0u; k < grid.m_Nz; ++k)
    {
        for (ClosestTriUniformGrid::CellIndex j = 0u; j < grid.m_Ny; ++j)
        {
            for (ClosestTriUniformGrid::CellIndex i = 0u; i < grid.m_Nx; ++i)
            {
                CellKey cellKey;
                if (!ComputeCellKeyFromIndex(
                    Nxyz, ClosestTriUniformGrid::ToIndex3(i, j, k), cellKey))
                    return BuilderStatus::eBUILDER_STATUS_INVALID_INDEX;

                TriKeyArray cellTris;// = triCells.at(cellKey).triIndices;

                // look through the region by expanding it while there are no tris found
                GridBorderRegionHelper testRegion(grid.m_Nx, grid.m_Ny, grid.m_Nz);
                testRegion.Reset(i, j, k);
                bool found = false;
                while (!found)
                {
                    CellKeyArray cellRegionKeys;
                    if (!testRegion.GetCellKeysFromRegion(*this, cellRegionKeys))
                        return BuilderStatus::eBUILDER_STATUS_INVALID_INDEX;

                    for (CellKey regionKey : cellRegionKeys)
                    {
                        const TriKeyArray& origTriIndices = triCells.at(regionKey).triIndices;
                        if (!origTriIndices.empty())
                        {
                            found = true;
                            cellTris.insert(cellTris.end(), origTriIndices.begin(), origTriIndices.end());
                        }
                    }

                    if (testRegion.hasFinished())
                        break;

                    // need to expand if not found any tris
                    if (!found)
                        testRegion.Expand();
                }

                // need to one last pass
                if (found)
                {
                    testRegion.Expand();

                    CellKeyArray cellRegionKeys;
                    if (!testRegion.GetCellKeysFromRegion(*this, cellRegionKeys))
                        return BuilderStatus::eBUILDER_STATUS_INVALID_INDEX;

                    for (CellKey regionKey : cellRegionKeys)
                    {
                        const TriKeyArray& origTriIndices = triCells.at(regionKey).triIndices;
                        if (!origTriIndices.empty())
                        {
                            found = true;
                            cellTris.insert(cellTris.end(), origTriIndices.begin(), origTriIndices.end());
                        }
                    }
                }

                if (!cellTris.empty())
                {
                    // get unique indices
                    std::sort(cellTris.begin(), cellTris.end());
                    auto last = std::unique(cellTris.begin(), cellTris.end());
                    cellTris.erase(last, cellTris.end());
                    //cellTris.shrink_to_fit();

                    // check if we need to expand the bit stream
                    if (writer.GetCurrentWritePos() + cellTris.size() * grid.m_indexBitWidth + 64u >=
                        grid.m_indexBitStream.GetSizeInBits())
                        grid.m_indexBitStream.Resize(2 * grid.m_indexBitStream.GetSizeInBytes());

                    // now store the indices to the bit stream
                    grid.m_indexCells.at(cellKey) = writer.GetCurrentWritePos();
                    for (TriKey t : cellTris)
                        writer.WriteNext(t);
                }
            }
        }
    }

    // clear temp data
    triCells.clear();
    // triCells.shrink_to_fit();

    // finalize the bit stream
    grid.m_lastBitPos = writer.GetCurrentWritePos();
    writer.Finalize();
    grid.m_indexBitStream.Pack();

    // copy params
    grid.m_cellWidth = cellWidth;
    grid.m_gridBBoxWorldSpace = gridBox;
    grid.m_vertices = std::move(vertices);
    grid.m_tris = std::move(tris);

    Clear();

    return BuilderStatus::eBUILDER_STATUS_SUCCESS;
}

ClosestTriUniformGrid::Builder::BuilderStatus  
ClosestTriUniformGrid::Builder::AddTriMesh(
    const std::vector<float>& positions, const std::vector<uint32_t>& indices)
{
    return AddTriMesh(positions.data(), (uint32_t)positions.size(),
                    indices.data(), indices.size(),
                    3u);
}

ClosestTriUniformGrid::Builder::BuilderStatus 
ClosestTriUniformGrid::Builder::AddTriMesh(
    const float* positions, uint32_t posCount, 
    const uint32_t* indices, uint32_t idxCount,
    uint32_t posStride)
{
    if (!vertices.empty() || !tris.empty())
        return BuilderStatus::eBUILDER_STATUS_NONEMPTY_MESH;

    // vertices and indices arrays need to be multiples of 3
    if (posCount % 3u != 0 || idxCount % 3u != 0 || posStride < 3u)
        return BuilderStatus::eBUILDER_STATUS_INVALID_PARAM_MESH;
    
    // gather vertices first
    vertices.reserve(posCount / 3u);
    for (size_t i = 0u; i < posCount; i += posStride)
    {
        Vector3 v(positions[i], positions[i+1], positions[i+2]);

        // check if point is in grid bounding box
        if (!gridBox.Contains(v))
            return BuilderStatus::eBUILDER_STATUS_POINT_OUT_OF_GRID;

        // transform vertex to grid space
        v.Sub(gridBox.min);

        // add to storage
        vertices.push_back(v);
    }
    vertices.shrink_to_fit();

    // add tris
    tris.reserve(idxCount / 3u);
    for (size_t triv = 0u; triv < idxCount; triv += 3u)
    {
        // add new tri info
        const uint32_t idx0 = indices[triv];
        const uint32_t idx1 = indices[triv + 1];
        const uint32_t idx2 = indices[triv + 2];

        TriInfo info;
        CreateTriInfoFromTriIndices(*this, idx0, idx1, idx2, info);
        TriKey triKey = AddTriInfo(*this, info);

        // get vertices for the tri
        const Vector3& v0 = vertices[idx0];
        const Vector3& v1 = vertices[idx1];
        const Vector3& v2 = vertices[idx2];

        // compute aabox
        AxisAlignedBoundingBox triBox;
        triBox.Reset();
        triBox.AddPoint(v0);
        triBox.AddPoint(v1);
        triBox.AddPoint(v2);

        // get min & max map indices
        CellIndex iMin, jMin, kMin;
        CellIndex iMax, jMax, kMax;
        if (!ComputeIndexFromPointGridSpace(cellWidth, triBox.min, iMin, jMin, kMin))
            return BuilderStatus::eBUILDER_STATUS_POINT_OUT_OF_GRID;
        if (!ComputeIndexFromPointGridSpace(cellWidth, triBox.max, iMax, jMax, kMax))
            return BuilderStatus::eBUILDER_STATUS_POINT_OUT_OF_GRID;

        // loop all cells and add tri if overlapping
        for (ClosestTriUniformGrid::CellIndex i = iMin; i <= iMax; ++i)
        {
            for (ClosestTriUniformGrid::CellIndex j = jMin; j <= jMax; ++j)
            {
                for (ClosestTriUniformGrid::CellIndex k = kMin; k <= kMax; ++k)
                {
                    // do the box-tri overlap test in grid space
                    AxisAlignedBoundingBox cellBox;
                    if (ComputeCelBBoxGridSpace(i, j, k, cellBox) != BuilderStatus::eBUILDER_STATUS_SUCCESS)
                        return BuilderStatus::eBUILDER_STATUS_INVALID_INDEX;

                    if (IntersectionQuery::OverlapAxisAlignedBoxTri(cellBox, v0, v1, v2))
                    {
                        ClosestTriUniformGrid::CellKey cellKey;
                        if (!ComputeCellKeyFromIndex(
                            Nxyz, ClosestTriUniformGrid::ToIndex3(i, j, k), cellKey))
                            return BuilderStatus::eBUILDER_STATUS_INVALID_INDEX;

                        AddTriToBucket(*this, cellKey, triKey);
                    }
                }
            }
        }
    }
    tris.shrink_to_fit();

    return BuilderStatus::eBUILDER_STATUS_SUCCESS;
}

ClosestTriUniformGrid::Builder::BuilderStatus
ClosestTriUniformGrid::Builder::GetOverlappingTrisOnCell(CellKey key, TriKeyArray& triIndices) const
{
    if (key >= (CellKey)triCells.size())
        return BuilderStatus::eBUILDER_STATUS_INVALID_INDEX;

    triIndices.clear();

    const TriCellBucket& bucket = triCells[key];
    for (TriKey triKey : bucket.triIndices)
        triIndices.push_back(triKey);

    return BuilderStatus::eBUILDER_STATUS_SUCCESS;
}

}