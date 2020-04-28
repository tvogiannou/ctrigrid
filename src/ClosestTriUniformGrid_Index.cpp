
#include <ctrigrid/ClosestTriUniformGrid.h>

#include <ctrigrid/Compiler.h>


namespace ctrigrid
{

bool 
ClosestTriUniformGrid::ComputeCellKeyFromIndex(
    const CellIndex3& Nxyz, const CellIndex3& ijk, CellKey& key)
{
    CellIndex Nx, Ny, Nz;
    CellIndex i, j, k;

    std::tie(Nx, Ny, Nz) = Nxyz;
    std::tie(i, j, k) = ijk;

    if (i >= Nx || j >= Ny || k >= Nz)
        return false;

    //if (m_cells.empty())
    //    return false;

    key = i + Nx * j + Nx * Ny * k;

    return true;
}

bool 
ClosestTriUniformGrid::ComputeIndexFromCellKey(
    const CellIndex3& Nxyz, CellKey key,
    CellIndex& i, CellIndex& j, CellIndex& k)
{
    CellIndex Nx, Ny, Nz;
    std::tie(Nx, Ny, Nz) = Nxyz;

    if (Nx == 0u || Ny == 0u || Nz == 0u)
        return false;

    k = (CellIndex)(key / (Nx * Ny));
    key = (CellIndex)(key % (Nx * Ny));
    j = (CellIndex)(key / Nx);
    i = (CellIndex)(key % Nx);

    return true;
}

bool 
ClosestTriUniformGrid::ComputeIndexFromPointGridSpace(
    float cellWidth, const Vector3& point, 
    CellIndex& i, CellIndex& j, CellIndex& k)
{
    if (cellWidth <= 0.f)
        return false;

    Vector3 c = point;
    c.Div(Vector3(cellWidth, cellWidth, cellWidth));

    i = (CellIndex)c.x;
    j = (CellIndex)c.y;
    k = (CellIndex)c.z;

    return true;
}

bool 
ClosestTriUniformGrid::ComputeCellKeyFromPoint(
    const CellIndex3& Nxyz, float cellWidth, const AxisAlignedBoundingBox& gridBox,
    const Vector3& point, CellKey& key)
{
    if (!gridBox.Contains(point))
        return false;

    // transform to grid space
    Vector3 c = point;
    c.Sub(gridBox.min);

    // compute indices
    CellIndex i, j, k;
    if (!ComputeIndexFromPointGridSpace(cellWidth, c, i, j, k))
        return false;

    // compute key
    if (!ComputeCellKeyFromIndex(Nxyz, ToIndex3(i, j, k), key))
        return false;

    return true;
}
}