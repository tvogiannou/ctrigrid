#pragma once

#include <ctrigrid/Compiler.h>
#include <ctrigrid/Vector3.h>


namespace ctrigrid
{

struct AxisAlignedBoundingBox
{
    Vector3 min;
    Vector3 max;

    // create/merge
    void Reset()
    {
        min = Vector3(FLT_MAX, FLT_MAX, FLT_MAX);
        max = Vector3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }
    void AddPoint(const Vector3& p);
    void AddAxisAlignedBoundinBox(const AxisAlignedBoundingBox& bbox);

    // get info
    Vector3 ComputeExtends() const;	// return half extends for each axis
    Vector3 ComputeCenter() const;	// return the center of the box

    // query
    bool Contains(const Vector3& p) const;
};

}