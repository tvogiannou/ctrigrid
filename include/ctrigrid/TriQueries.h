#pragma once

#include <ctrigrid/Compiler.h>
#include <ctrigrid/Vector3.h>
#include <ctrigrid/AxisAlignedBoundingBox.h>


namespace ctrigrid
{

struct IntersectionQuery
{
    static bool OverlapAxisAlignedBoxTri(
        const AxisAlignedBoundingBox& aabox,
        const Vector3& triV0, const Vector3& triV1, const Vector3& triV2);
};

struct ClosestDistanceQuery
{
    static bool ClosestPointOnTri(
        const Vector3& point, 
        const Vector3& triV0, const Vector3& triV1, const Vector3& triV2,
        Vector3& closestPoint);

    static bool ClosestPointOnAxisAlignedBox(
        const Vector3& point,
        const AxisAlignedBoundingBox& bbox,
        Vector3& closestPoint);
};

}