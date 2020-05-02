
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
ClosestTriUniformGrid::FindClosestPointOnTris(
    const Vector3& p, Vector3& closestPoint, TriKey& triKey, bool forceInGrid) const
{
    AxisAlignedBoundingBox gridBox = GetGridAABoxWorldSpace();
    CellKey cellKey;
    if (!ComputeCellKeyFromPoint(
            ToIndex3(m_Nx, m_Ny, m_Nz), m_cellWidth, gridBox, p, cellKey))
    {
        if (forceInGrid)    // force the result to be inside the grid
            return false;

        // if the point is outside the grid, just fall back to finding the closest cell
        // NOTE: this is an approximation and may not always give correct results
        Vector3 closestCellPoint;
        ClosestDistanceQuery::ClosestPointOnAxisAlignedBox(p, gridBox, closestCellPoint);

        // make sure closest point is inside grid by "nudging" it towards the grid center
        const Vector3 c = gridBox.ComputeCenter();
        Vector3 d = c;
        d.Sub(closestCellPoint);
        d.Normalize();
        closestCellPoint.MulAdd(m_cellWidth * .5f, d);

        if (!ComputeCellKeyFromPoint(
            ToIndex3(m_Nx, m_Ny, m_Nz), m_cellWidth, gridBox, closestCellPoint, cellKey))
        {
            // Should never get here!
            CTRIGRID_ASSERT(false);
            return false;
        }
    }

    Vector3 localP = p;
    localP.Sub(gridBox.min);

    CTRIGRID_ASSERT(cellKey < m_indexCells.size());
    const BitStreamBuffer::BufferIndex startPos = m_indexCells[cellKey];
    const BitStreamBuffer::BufferIndex endPos = 
        cellKey + 1u == m_indexCells.size() ? m_lastBitPos : m_indexCells[cellKey + 1u];
    BitStreamReader reader(m_indexBitStream, m_indexBitWidth);
    reader.Begin(startPos, endPos);

#ifdef CTRIGRID_GRID_BOUNDS_CHECK
    TriKeyArray triIndices;
    while (!reader.Finished())
        triIndices.push_back((TriKey)reader.Next());

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
            const TriKey triIndex0 = triIndices[(size_t)wideI];
            const TriKey triIndex1 = triIndices[(size_t)wideI + 1];
            const TriKey triIndex2 = triIndices[(size_t)wideI + 2];
            const TriKey triIndex3 = triIndices[(size_t)wideI + 3];

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
            const TriKey triIndex = triIndices[i];

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
        for (TriKey triIndex : triIndices)
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
        const TriKey triIndex = triIndices[i];
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
        const TriKey triIndex = (TriKey)reader.Next();

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
    closestPoint.Add(gridBox.min);

    return true;
}

ClosestTriUniformGrid::MemoryStats
ClosestTriUniformGrid::ComputeMemFootprint() const
{
    MemoryStats stats = { 0u, 0u, 0u, 0u };

    stats.verticesAllocMem = m_vertices.capacity() * sizeof(Vector3);
    stats.trisAllocMem += m_tris.capacity() * sizeof(TriInfo);
    stats.cellsAllocMem += m_indexCells.capacity() * sizeof(BitStreamBuffer::BufferIndex);
    stats.cellIndicesAllocMem = 0u;
    stats.cellIndicesAllocMem += m_indexBitStream.GetSizeInBytes();

    return stats;
}

ClosestTriUniformGrid::Builder::BuilderStatus 
ClosestTriUniformGrid::GetClosestTrisOnCell(CellKey key, TriKeyArray& triIndices) const
{
    if (key >= (CellKey)m_indexCells.size())
        return Builder::BuilderStatus::eBUILDER_STATUS_SUCCESS;

    triIndices.clear();

    const BitStreamBuffer::BufferIndex startPos = m_indexCells[key];
    const BitStreamBuffer::BufferIndex endPos =
        key + 1u == m_indexCells.size() ? m_lastBitPos : m_indexCells[key + 1u];

    BitStreamReader reader(m_indexBitStream, m_indexBitWidth);
    reader.Begin(startPos, endPos);
    while (!reader.Finished())
        triIndices.push_back((TriKey)reader.Next());

    return Builder::BuilderStatus::eBUILDER_STATUS_SUCCESS;
}

bool 
ClosestTriUniformGrid::GetTriVerticesWorldSpace(
    TriKey triKey, Vector3& v0, Vector3& v1, Vector3& v2) const
{
    if (triKey >= (TriKey)m_tris.size())
        return false;

    const TriInfo& info = m_tris[triKey];
    CTRIGRID_ASSERT(info.idx0 < m_vertices.size());
    CTRIGRID_ASSERT(info.idx1 < m_vertices.size());
    CTRIGRID_ASSERT(info.idx2 < m_vertices.size());
    v0 = m_vertices[info.idx0];
    v1 = m_vertices[info.idx1];
    v2 = m_vertices[info.idx2];

    AxisAlignedBoundingBox gridBBoxWorldSpace = GetGridAABoxWorldSpace();
    v0.Add(gridBBoxWorldSpace.min);
    v1.Add(gridBBoxWorldSpace.min);
    v2.Add(gridBBoxWorldSpace.min);

    return true;
}

void 
ClosestTriUniformGrid::Clear()
{
    m_Nx = 0u;
    m_Ny = 0u;
    m_Nz = 0u;
    m_cellWidth = -1;

    m_vertices.clear();
    m_tris.clear();

    m_indexBitWidth = 0u;
    m_lastBitPos = 0u;
    m_indexBitStream.Clear();
    m_indexCells.clear();
}


}