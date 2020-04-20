

#include <ctrigrid/ClosestTriUniformGrid.h>

#include <ctrigrid/Compiler.h>
#include <ctrigrid/MathOptConfig.h>

#include <ctrigrid/TriQueries.h>



#ifdef CTRIGRID_GRID_BOUNDS_CHECK_SSE
    #include <ctrigrid/Vector4.h>
#endif


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
ClosestTriUniformGrid::ComputeCellKeyFromPoint(const Vector3& point, MapCellKeyType& key) const
{
    if (!m_gridBBoxWorldSpace.Contains(point))
        return false;

    // transform to grid space
    Vector3 c = point;
    c.Sub(m_gridBBoxWorldSpace.min);

    // compute indices
    MapIndexType i, j, k;
    if (!ComputeIndexFromPointGridSpace(c, i, j, k))
        return false;

    // compute key
    if (!ComputeCellKeyFromIndex(i, j, k, key))
        return false;

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

bool 
ClosestTriUniformGrid::FindClosestPointOnTris(
    const Vector3& p, Vector3& closestPoint, MapTriKeyType& triKey, bool forceInGrid) const
{
    MapCellKeyType cellKey;
    if (!ComputeCellKeyFromPoint(p, cellKey))
    {
        if (forceInGrid)    // force the result to be inside the grid
            return false;

        // if the point is outside the grid, just fall back to finding the closest cell
        // NOTE: this is an approximation and may not always give correct results
        Vector3 closestCellPoint;
        ClosestDistanceQuery::ClosestPointOnAxisAlignedBox(p, m_gridBBoxWorldSpace, closestCellPoint);

        // make sure closest point is inside grid by "nudging" it towards the grid center
        const Vector3 c = m_gridBBoxWorldSpace.ComputeCenter();
        Vector3 d = c;
        d.Sub(closestCellPoint);
        d.Normalize();
        closestCellPoint.MulAdd(m_cellWidth * .5f, d);

        if (!ComputeCellKeyFromPoint(closestCellPoint, cellKey))
        {
            // Should never get here!
            CTRIGRID_ASSERT(false);
            return false;
        }
    }

    Vector3 localP = p;
    localP.Sub(m_gridBBoxWorldSpace.min);

    CTRIGRID_ASSERT(cellKey < m_indexCells.size());
    //const MapTriKeyArrayType& tris = m_triCells[cellKey].triIndices;
    const BitStreamBuffer::BufferIndexType startPos = m_indexCells[cellKey];
    const BitStreamBuffer::BufferIndexType endPos = 
        cellKey + 1u == m_indexCells.size() ? m_lastBitPos : m_indexCells[cellKey + 1u];
    BitStreamReader reader(m_indexBitStream, m_indexBitWidth);
    reader.Begin(startPos, endPos);

#ifdef CTRIGRID_GRID_BOUNDS_CHECK
    MapTriKeyArrayType triIndices;
    while (!reader.Finished())
        triIndices.push_back((MapTriKeyType)reader.Next());

    // first compute bounds for all tris
    struct Bounds { float min, max; };
    std::vector<Bounds> triBounds;
    triBounds.reserve(triIndices.size());

    // keep track of the closest bound
    Bounds closestBound = { std::numeric_limits<float>::max(), 0.f };
    {
#ifdef CTRIGRID_GRID_BOUNDS_CHECK_SSE
        const __m128 pX = _mm_set_ps1(localP.x);
        const __m128 pY = _mm_set_ps1(localP.y);
        const __m128 pZ = _mm_set_ps1(localP.z);

        // do 4 spheres at a time
        const int64_t batchCount = (int64_t)triIndices.size() - 4;
        int64_t wideI = 0;
        for (wideI = 0; wideI < batchCount; wideI +=4u)
        {
            const MapTriKeyType triIndex0 = triIndices[(size_t)wideI];
            const MapTriKeyType triIndex1 = triIndices[(size_t)wideI + 1];
            const MapTriKeyType triIndex2 = triIndices[(size_t)wideI + 2];
            const MapTriKeyType triIndex3 = triIndices[(size_t)wideI + 3];

            const TriInfo& info0 = m_tris[triIndex0];
            const TriInfo& info1 = m_tris[triIndex1];
            const TriInfo& info2 = m_tris[triIndex2];
            const TriInfo& info3 = m_tris[triIndex3];

            __m128 X = _mm_set_ps(info0.sphereCenter.x, info1.sphereCenter.x, info2.sphereCenter.x, info3.sphereCenter.x);
            __m128 Y = _mm_set_ps(info0.sphereCenter.y, info1.sphereCenter.y, info2.sphereCenter.y, info3.sphereCenter.y);
            __m128 Z = _mm_set_ps(info0.sphereCenter.z, info1.sphereCenter.z, info2.sphereCenter.z, info3.sphereCenter.z);
        
            X = _mm_sub_ps(X, pX);
            Y = _mm_sub_ps(Y, pY);
            Z = _mm_sub_ps(Z, pZ);

            // compute distances
            __m128 S = _mm_mul_ps(X, X);
            S = _mm_add_ps(_mm_mul_ps(Y, Y), S);
            S = _mm_add_ps(_mm_mul_ps(Z, Z), S);
            S = _mm_sqrt_ps(S);

            Vector4 dists;
            _mm_store_ps(dists.m_elems, S);

            Bounds newBounds0 = { dists.x - info0.sphereRadius, dists.x + info0.sphereRadius };
            Bounds newBounds1 = { dists.x - info1.sphereRadius, dists.x + info1.sphereRadius };
            Bounds newBounds2 = { dists.x - info2.sphereRadius, dists.x + info2.sphereRadius };
            Bounds newBounds3 = { dists.x - info3.sphereRadius, dists.x + info3.sphereRadius };

            if (newBounds0.min < 0.f) newBounds0.min = 0.f;
            if (newBounds1.min < 0.f) newBounds1.min = 0.f;
            if (newBounds2.min < 0.f) newBounds2.min = 0.f;
            if (newBounds3.min < 0.f) newBounds3.min = 0.f;

            triBounds.push_back(newBounds0);
            triBounds.push_back(newBounds1);
            triBounds.push_back(newBounds2);
            triBounds.push_back(newBounds3);

            if (newBounds0.min < closestBound.min) closestBound = newBounds0;
            if (newBounds1.min < closestBound.min) closestBound = newBounds1;
            if (newBounds2.min < closestBound.min) closestBound = newBounds2;
            if (newBounds3.min < closestBound.min) closestBound = newBounds3;
        }

        // do the rest 
        for (size_t i = (size_t)wideI; i < triIndices.size(); ++i)
        {
            const MapTriKeyType triIndex = triIndices[i];

            CTRIGRID_ASSERT(triIndex < m_tris.size());
            const TriInfo& info = m_tris[triIndex];

            Vector3 d = info.sphereCenter;
            d.Sub(localP);
            const float dist = d.Norm();

            Bounds newBounds = { dist - info.sphereRadius, dist + info.sphereRadius };
            if (newBounds.min < 0.f) newBounds.min = 0.f;
            triBounds.push_back(newBounds);

            if (newBounds.min < closestBound.min)
                closestBound = newBounds;
        }

#else
        for (MapTriKeyType triIndex : triIndices)
        {
            CTRIGRID_ASSERT(triIndex < m_tris.size());
            const TriInfo& info = m_tris[triIndex];

            Vector3 d = info.sphereCenter;
            d.Sub(localP);
            const float dist = d.Norm();

            Bounds newBounds = { dist - info.sphereRadius, dist + info.sphereRadius };
            if (newBounds.min < 0.f) newBounds.min = 0.f;
            triBounds.push_back(newBounds);

            if (newBounds.min < closestBound.min)
                closestBound = newBounds;
        }

#endif
    }

    // do a second pass to find the closest tri but only check tris that are included in the closest bound
    float distSqr = std::numeric_limits<float>::max();   // max dist
    for (size_t i = 0; i < triIndices.size(); ++i)
    {
        const MapTriKeyType triIndex = triIndices[i];
        const TriInfo& info = m_tris[triIndex];
        const Bounds& b = triBounds[i];

        if (b.min <= closestBound.max)
        {
            CTRIGRID_ASSERT(info.idx0 < m_vertices.size());
            CTRIGRID_ASSERT(info.idx1 < m_vertices.size());
            CTRIGRID_ASSERT(info.idx2 < m_vertices.size());
            const Vector3& v0 = m_vertices[info.idx0];
            const Vector3& v1 = m_vertices[info.idx1];
            const Vector3& v2 = m_vertices[info.idx2];
            Vector3 tempClosestP;
            ClosestDistanceQuery::ClosestPointOnTri(localP, v0, v1, v2, tempClosestP);

            // keep if it is the closest point till now
            Vector3 delta = tempClosestP;
            delta.Sub(localP);
            const float d = delta.Dot();
            if (d < distSqr)
            {
                closestPoint = tempClosestP;
                distSqr = d;
                triKey = triIndex;
            }
        }
    }

#else
    // find the tri with the smaller distance
    float distSqr = std::numeric_limits<float>::max();   // max dist
    while (!reader.Finished())
    {
        const MapTriKeyType triIndex = (MapTriKeyType)reader.Next();

        // run a closest point query
        CTRIGRID_ASSERT(triIndex < m_tris.size());
        const TriInfo& info = m_tris[triIndex];
        CTRIGRID_ASSERT(info.idx0 < m_vertices.size());
        CTRIGRID_ASSERT(info.idx1 < m_vertices.size());
        CTRIGRID_ASSERT(info.idx2 < m_vertices.size());
        const Vector3& v0 = m_vertices[info.idx0];
        const Vector3& v1 = m_vertices[info.idx1];
        const Vector3& v2 = m_vertices[info.idx2];
        Vector3 tempClosestP;
        ClosestDistanceQuery::ClosestPointOnTri(localP, v0, v1, v2, tempClosestP);

        // keep if it is the closest point till now
        Vector3 delta = tempClosestP;
        delta.Sub(localP);
        const float d = delta.Dot();
        if (d < distSqr)
        {
            closestPoint = tempClosestP;
            distSqr = d;
            triKey = triIndex;
        }
    }
#endif

    // transform back to world space
    closestPoint.Add(m_gridBBoxWorldSpace.min);

    return true;
}

ClosestTriUniformGrid::MemoryStats
ClosestTriUniformGrid::ComputeMemFootprint() const
{
    MemoryStats stats = { 0u, 0u, 0u, 0u };

    stats.verticesAllocMem = m_vertices.capacity() * sizeof(Vector3);
    stats.trisAllocMem += m_tris.capacity() * sizeof(TriInfo);
    stats.cellsAllocMem += m_triCells.capacity() * sizeof(TriCellBucket);
    stats.cellsAllocMem += m_indexCells.capacity() * sizeof(BitStreamBuffer::BufferIndexType);
    stats.cellIndicesAllocMem = 0u;
    for (const TriCellBucket& cell : m_triCells)
        stats.cellIndicesAllocMem += cell.triIndices.capacity() * sizeof(MapTriKeyType);
    stats.cellIndicesAllocMem += m_indexBitStream.GetSizeInBytes();

    return stats;
}

bool 
ClosestTriUniformGrid::GetClosestTrisOnCell(MapCellKeyType key, MapTriKeyArrayType& triIndices) const
{
    if (key >= (MapCellKeyType)m_indexCells.size())
        return false;

    triIndices.clear();

    const BitStreamBuffer::BufferIndexType startPos = m_indexCells[key];
    const BitStreamBuffer::BufferIndexType endPos =
        key + 1u == m_indexCells.size() ? m_lastBitPos : m_indexCells[key + 1u];

    BitStreamReader reader(m_indexBitStream, m_indexBitWidth);
    reader.Begin(startPos, endPos);
    while (!reader.Finished())
        triIndices.push_back((MapTriKeyType)reader.Next());

    return true;
}

bool
ClosestTriUniformGrid::GetOverlappingTrisOnCell(MapCellKeyType key, MapTriKeyArrayType& triIndices) const
{
    if (key >= (MapCellKeyType)m_triCells.size())
        return false;

    triIndices.clear();

    const TriCellBucket& bucket = m_triCells[key];
    for (MapTriKeyType triKey : bucket.triIndices)
        triIndices.push_back(triKey);

    return true;
}

bool 
ClosestTriUniformGrid::GetTriVerticesWorldSpace(
    MapTriKeyType triKey, Vector3& v0, Vector3& v1, Vector3& v2) const
{
    if (triKey >= (MapTriKeyType)m_tris.size())
        return false;

    const TriInfo& info = m_tris[triKey];
    CTRIGRID_ASSERT(info.idx0 < m_vertices.size());
    CTRIGRID_ASSERT(info.idx1 < m_vertices.size());
    CTRIGRID_ASSERT(info.idx2 < m_vertices.size());
    v0 = m_vertices[info.idx0];
    v1 = m_vertices[info.idx1];
    v2 = m_vertices[info.idx2];

    v0.Add(m_gridBBoxWorldSpace.min);
    v1.Add(m_gridBBoxWorldSpace.min);
    v2.Add(m_gridBBoxWorldSpace.min);

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