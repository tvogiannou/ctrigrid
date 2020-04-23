

#include <ctrigrid/ClosestTriUniformGrid.h>

#include <ctrigrid/Compiler.h>
#include <ctrigrid/TriQueries.h>


namespace ctrigrid
{

// Helper representing the border of a region ("shell") in the grid
// TODO: change this so it keeps tracks the cells that have been checked while expanding
struct GridBorderRegionHelper
{
    GridBorderRegionHelper(
        ClosestTriUniformGrid::MapIndexType _Nx,
        ClosestTriUniformGrid::MapIndexType _Ny,
        ClosestTriUniformGrid::MapIndexType _Nz) 
        : Nx(_Nx), Ny(_Ny), Nz(_Nz)
    {}

    const ClosestTriUniformGrid::MapIndexType Nx;
    const ClosestTriUniformGrid::MapIndexType Ny;
    const ClosestTriUniformGrid::MapIndexType Nz;

    ClosestTriUniformGrid::MapIndexType iMin, iMax;
    ClosestTriUniformGrid::MapIndexType jMin, jMax;
    ClosestTriUniformGrid::MapIndexType kMin, kMax;

    void Reset(
        ClosestTriUniformGrid::MapIndexType i,
        ClosestTriUniformGrid::MapIndexType j,
        ClosestTriUniformGrid::MapIndexType k)
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
        const ClosestTriUniformGrid& grid,
        ClosestTriUniformGrid::MapCellKeyArrayType& cellRegionKeys)
    {
        // special case
        if (iMin == iMax && jMin == jMax && kMin == kMax)
        {
            ClosestTriUniformGrid::MapCellKeyType key;
            if (!grid.ComputeCellKeyFromIndex(iMin, jMin, kMin, key))
                return false;
            cellRegionKeys.push_back(key);

            return true;
        }

        // +-z sides of border
        for (ClosestTriUniformGrid::MapIndexType i = iMin; i <= iMax; ++i)
        {
            for (ClosestTriUniformGrid::MapIndexType j = jMin; j <= jMax; ++j)
            {
                ClosestTriUniformGrid::MapCellKeyType key;
                if (!grid.ComputeCellKeyFromIndex(i, j, kMin, key)) 
                    return false;
                cellRegionKeys.push_back(key);

                if (kMin != kMax)
                {
	                if (!grid.ComputeCellKeyFromIndex(i, j, kMax, key))
	                    return false;
	                cellRegionKeys.push_back(key);
                }
            }
        }

        for (ClosestTriUniformGrid::MapIndexType k = kMin + 1u; k < kMax; ++k)
        {
            // =-y sides
            for (ClosestTriUniformGrid::MapIndexType i = iMin; i <= iMax; ++i)
            {
                ClosestTriUniformGrid::MapCellKeyType key;
                if (!grid.ComputeCellKeyFromIndex(i, jMin, k, key))
                    return false;
                cellRegionKeys.push_back(key);

                if (jMin != jMax)
                {
                    if (!grid.ComputeCellKeyFromIndex(i, jMax, k, key))
                        return false;
                    cellRegionKeys.push_back(key);
                }
            }

            // +-x sides
            for (ClosestTriUniformGrid::MapIndexType j = jMin; j <= jMax; ++j)
            {
                ClosestTriUniformGrid::MapCellKeyType key;
                if (!grid.ComputeCellKeyFromIndex(iMin, j, k, key))
                    return false;
                cellRegionKeys.push_back(key);

                if (iMin != iMax)
                {
                    if (!grid.ComputeCellKeyFromIndex(iMax, j, k, key))
                        return false;
                    cellRegionKeys.push_back(key);
                }
            }
        }

        return true;
    }
};

bool 
ClosestTriUniformGrid::ComputeCellKeyFromIndex(
    MapIndexType i, MapIndexType j, MapIndexType k, MapCellKeyType& key) const
{
    if (i >= m_Nx || j >= m_Ny || k >= m_Nz)
        return false;

    //if (m_cells.empty())
    //    return false;

    key = i + m_Nx * j + m_Nx * m_Ny * k;

    return true;
}

bool 
ClosestTriUniformGrid::ComputeIndexFromCellKey(
    MapCellKeyType key, MapIndexType& i, MapIndexType& j, MapIndexType& k) const
{
    if (m_Nx == 0u || m_Ny == 0u || m_Nz == 0u)
        return false;

    k = (MapIndexType)(key / (m_Nx * m_Ny));
    key = (MapIndexType)(key % (m_Nx * m_Ny));
    j = (MapIndexType)(key / m_Nx);
    i = (MapIndexType)(key % m_Nx);

    return true;
}

bool 
ClosestTriUniformGrid::ComputeIndexFromPointGridSpace(
    const Vector3& point, MapIndexType& i, MapIndexType& j, MapIndexType& k) const
{
    if (m_cellWidth <= 0.f)
        return false;

    Vector3 c = point;
    c.Div(Vector3(m_cellWidth, m_cellWidth, m_cellWidth));

    i = (MapIndexType)c.x;
    j = (MapIndexType)c.y;
    k = (MapIndexType)c.z;

    return true;
}

bool 
ClosestTriUniformGrid::ComputeCelBBoxWorldSpace(
    MapIndexType i, MapIndexType j, MapIndexType k, AxisAlignedBoundingBox& bbox) const
{
    if (!ComputeCelBBoxGridSpace(i, j, k, bbox))
        return false;

    // add grid origin
    bbox.min.Add(m_gridBBoxWorldSpace.min);
    bbox.max.Add(m_gridBBoxWorldSpace.min);

    return true;
}

bool 
ClosestTriUniformGrid::ComputeCelBBoxGridSpace(MapIndexType i, MapIndexType j, MapIndexType k, AxisAlignedBoundingBox& bbox) const
{
    if (i >= m_Nx || j >= m_Ny || k >= m_Nz)
        return false;

    // compute start and end points of cell bbox
    Vector3 p0(((float)i) * m_cellWidth, ((float)j) * m_cellWidth, ((float)k) * m_cellWidth);
    Vector3 p1 = p0;
    p1.Add(m_cellWidth);

    bbox.Reset();
    bbox.AddPoint(p0);
    bbox.AddPoint(p1);

    return true;
}

bool 
ClosestTriUniformGrid::Init(const InitInfo& info)
{
    if (info.Nx == 0u || info.Ny == 0u || info.Nz == 0u)
        return false;
    if (info.cellWidth <= 0.f)
        return false;

    m_Nx = info.Nx;
    m_Ny = info.Ny;
    m_Nz = info.Nz;
    m_cellWidth = info.cellWidth;

    Vector3 maxExtend(((float)m_Nx) * m_cellWidth, 
                      ((float)m_Ny) * m_cellWidth, 
                      ((float)m_Nz) * m_cellWidth);
    maxExtend.Add(info.origin);
    
    m_gridBBoxWorldSpace.Reset();
    m_gridBBoxWorldSpace.AddPoint(info.origin);
    m_gridBBoxWorldSpace.AddPoint(maxExtend);

    return true;
}

bool 
ClosestTriUniformGrid::Clear()
{
    m_Nx = 0u;
    m_Ny = 0u;
    m_Nz = 0u;
    m_cellWidth = -1;
    m_gridBBoxWorldSpace.Reset();

    m_tris.clear();
    m_triCells.clear();
    
    m_indexBitWidth = 0u;
    m_lastBitPos = 0u;
    m_indexCells.clear();
    m_indexBitStream.Clear();
    m_indexBitStream.Pack();

    return true;
}

bool 
ClosestTriUniformGrid::BeginGridSetup()
{
    if (!m_vertices.empty() || !m_tris.empty() || !m_triCells.empty())
        return false;

    if (m_Nx == 0u || m_Ny == 0u || m_Nz == 0u)
        return false;

    m_triCells.resize(m_Nx * m_Ny * m_Nz);

    return true;
}

bool 
ClosestTriUniformGrid::FinalizeGridSetup()
{
    if (m_tris.empty() || m_triCells.empty())
        return false;

    if (m_Nx == 0u || m_Ny == 0u || m_Nz == 0u)
        return false;

    // figure out the width required for the number of tris we got
    {
        size_t maxTriIndex = m_tris.size();
        m_indexBitWidth = 1;
        while (maxTriIndex >>= 1u) ++m_indexBitWidth;
    }
    
    // setup the bit stream for the indices
    m_indexBitStream.Clear();
    BitStreamWriter writer(m_indexBitStream, m_indexBitWidth);
    writer.Begin(m_tris.size() + sizeof(uint64_t) + 1u);    // rough estimate for initialization
    m_indexCells.clear();
    m_indexCells.resize(m_Nx * m_Ny * m_Nz);
    m_indexCells.shrink_to_fit();

    // loop through all the cells and find the nearest ones with tris
    for (ClosestTriUniformGrid::MapIndexType k = 0u; k < m_Nz; ++k)
    {
        for (ClosestTriUniformGrid::MapIndexType j = 0u; j < m_Ny; ++j)
        {
            for (ClosestTriUniformGrid::MapIndexType i = 0u; i < m_Nx; ++i)
            {
                MapCellKeyType cellKey;
                if (!ComputeCellKeyFromIndex(i, j, k, cellKey))
                    return false;

                MapTriKeyArrayType cellTris;// = m_triCells.at(cellKey).triIndices;

                // look through the region by expanding it while there are no tris found
                GridBorderRegionHelper testRegion(m_Nx, m_Ny, m_Nz);
                testRegion.Reset(i, j, k);
                bool found = false;
                while (!found)
                {
                    MapCellKeyArrayType cellRegionKeys;
                    if (!testRegion.GetCellKeysFromRegion(*this, cellRegionKeys))
                        return false;

                    for (MapCellKeyType regionKey : cellRegionKeys)
                    {
                        const MapTriKeyArrayType& origTriIndices = m_triCells.at(regionKey).triIndices;
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

                    MapCellKeyArrayType cellRegionKeys;
                    if (!testRegion.GetCellKeysFromRegion(*this, cellRegionKeys))
                        return false;

                    for (MapCellKeyType regionKey : cellRegionKeys)
                    {
                        const MapTriKeyArrayType& origTriIndices = m_triCells.at(regionKey).triIndices;
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
                    if (writer.GetCurrentWritePos() + cellTris.size() * m_indexBitWidth + 64u >=
                        m_indexBitStream.GetSizeInBits())
                        m_indexBitStream.Resize(2 * m_indexBitStream.GetSizeInBytes());

                    // now store the indices to the bit stream
                    m_indexCells.at(cellKey) = writer.GetCurrentWritePos();
                    for (MapTriKeyType t : cellTris)
                        writer.WriteNext(t);
                }
            }
        }
    }

    // clear temp data
    m_triCells.clear();
    m_triCells.shrink_to_fit();

    // finalize the bit stream
    m_lastBitPos = writer.GetCurrentWritePos();
    writer.Finalize();
    m_indexBitStream.Pack();

    return true;
}

bool 
ClosestTriUniformGrid::AddTriMesh(const std::vector<float>& vertices, const std::vector<uint32_t>& indices)
{
    // vertices and indices arrays need to be multiples of 3
    if (vertices.size() % 3u != 0 || indices.size() % 3u != 0)
        return false;

    // gather vertices first
    CTRIGRID_ASSERT(m_vertices.empty());
    m_vertices.reserve(vertices.size() / 3u);
    for (size_t i = 0u; i < vertices.size(); i += 3u)
    {
        Vector3 v(vertices[i], vertices[i+1], vertices[i+2]);

        // check if point is in grid bounding box
        if (!m_gridBBoxWorldSpace.Contains(v))
            return false;

        // transform vertex to grid space
        v.Sub(m_gridBBoxWorldSpace.min);

        // add to storage
        m_vertices.push_back(v);
    }
    m_vertices.shrink_to_fit();

    // add tris
    CTRIGRID_ASSERT(m_tris.empty());
    m_tris.reserve(indices.size() / 3u);
    for (size_t triv = 0u; triv < indices.size(); triv += 3u)
    {
        // add new tri info
        const uint32_t idx0 = indices[triv];
        const uint32_t idx1 = indices[triv + 1];
        const uint32_t idx2 = indices[triv + 2];

        TriInfo info;
        CreateTriInfoFromTriIndices(idx0, idx1, idx2, info);
        MapTriKeyType triKey = AddTriInfo(info);

        // get vertices for the tri
        const Vector3& v0 = m_vertices[idx0];
        const Vector3& v1 = m_vertices[idx1];
        const Vector3& v2 = m_vertices[idx2];

        // compute aabox
        AxisAlignedBoundingBox triBox;
        triBox.Reset();
        triBox.AddPoint(v0);
        triBox.AddPoint(v1);
        triBox.AddPoint(v2);

        // get min & max map indices
        MapIndexType iMin, jMin, kMin;
        MapIndexType iMax, jMax, kMax;
        if (!ComputeIndexFromPointGridSpace(triBox.min, iMin, jMin, kMin))
            return false;
        if (!ComputeIndexFromPointGridSpace(triBox.max, iMax, jMax, kMax))
            return false;

        // loop all cells and add tri if overlapping
        for (ClosestTriUniformGrid::MapIndexType i = iMin; i <= iMax; ++i)
        {
            for (ClosestTriUniformGrid::MapIndexType j = jMin; j <= jMax; ++j)
            {
                for (ClosestTriUniformGrid::MapIndexType k = kMin; k <= kMax; ++k)
                {
                    // do the box-tri overlap test in grid space
                    AxisAlignedBoundingBox cellBox;
                    if (!ComputeCelBBoxGridSpace(i, j, k, cellBox))
                        return false;

                    if (IntersectionQuery::OverlapAxisAlignedBoxTri(cellBox, v0, v1, v2))
                    {
                        ClosestTriUniformGrid::MapCellKeyType cellKey;
                        if (!ComputeCellKeyFromIndex(i, j, k, cellKey))
                            return false;

                        AddTriToBucket(cellKey, triKey);
                    }
                }
            }
        }
    }
    m_tris.shrink_to_fit();

    return true;
}

void 
ClosestTriUniformGrid::CreateTriInfoFromTriIndices(MapTriKeyType idx0, MapTriKeyType idx1, MapTriKeyType idx2, TriInfo& info)
{
    info.idx0 = idx0;
    info.idx1 = idx1;
    info.idx2 = idx2;

    // compute bounding sphere
    {
        CTRIGRID_ASSERT(info.idx0 < m_vertices.size());
        CTRIGRID_ASSERT(info.idx1 < m_vertices.size());
        CTRIGRID_ASSERT(info.idx2 < m_vertices.size());

        const Vector3& v0 = m_vertices[info.idx0];
        const Vector3& v1 = m_vertices[info.idx1];
        const Vector3& v2 = m_vertices[info.idx2];

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

ClosestTriUniformGrid::MapCellKeyType 
ClosestTriUniformGrid::AddTriInfo(const TriInfo& info)
{
    m_tris.emplace_back(info);

    return (ClosestTriUniformGrid::MapCellKeyType)(m_tris.size() - 1u);
}

void 
ClosestTriUniformGrid::AddTriToBucket(MapCellKeyType cellKey, MapTriKeyType triKey)
{
    TriCellBucket& bucket = m_triCells.at(cellKey);

    // TODO: keep sorted to optimize search
    if (std::find(bucket.triIndices.begin(), bucket.triIndices.end(), triKey) == bucket.triIndices.end())
        bucket.triIndices.push_back(triKey);
}

}