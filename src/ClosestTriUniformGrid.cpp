

#include <ctrigrid/ClosestTriUniformGrid.h>

#include <ctrigrid/Compiler.h>
#include <ctrigrid/MathOptConfig.h>

#include <ctrigrid/TriQueries.h>


#ifdef CTRIGRID_GRID_BOUNDS_CHECK_SSE
    #include <ctrigrid/Vector4.h>
#endif


namespace ctrigrid
{

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

}