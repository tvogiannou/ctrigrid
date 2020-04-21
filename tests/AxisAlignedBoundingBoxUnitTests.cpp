
#include <gtest/gtest.h>

#include <ctrigrid/AxisAlignedBoundingBox.h>

#include "MathTestUtils.h"



TEST(AxisAlignedBoundingBoxUnitTests, TestAxisAlignedBoundingBoxReset)
{
    using namespace ctrigrid;

    AxisAlignedBoundingBox aabb;
    aabb.Reset();

    // reset should result in min > max so that we can keep adding points
    EXPECT_GT(aabb.min.x, aabb.max.x);
    EXPECT_GT(aabb.min.y, aabb.max.y);
    EXPECT_GT(aabb.min.z, aabb.max.z);
}

TEST(AxisAlignedBoundingBoxUnitTests, TestAxisAlignedBoundingBoxAdd)
{
    using namespace ctrigrid;

    {
        const Vector3 low(-3.f, -2.f, -1.f);
        const Vector3 high(1.f, 2.f, 3.f);
        
        AxisAlignedBoundingBox aabb;
        aabb.Reset();
        aabb.AddPoint(high);
        
        // min & max should match
        mathtests::Utils::CheckValues(aabb.max, aabb.min);
        mathtests::Utils::CheckValues(aabb.max, high);

        aabb.AddPoint(low);
        mathtests::Utils::CheckValues(aabb.min, low);
        mathtests::Utils::CheckValues(aabb.max, high);
    }

    {
        const Vector3 low1(-3.f, -2.f, -1.f);
        const Vector3 low2(-5.f, 2.f, -1.f);
        const Vector3 high1(1.f, 2.f, 3.f);
        const Vector3 high2(1.f, 4.f, 1.f);

        AxisAlignedBoundingBox aabb1;
        AxisAlignedBoundingBox aabb2;

        aabb1.Reset();
        aabb1.AddPoint(low1);
        aabb1.AddPoint(high1);

        aabb2.Reset();
        aabb2.AddPoint(low2);
        aabb2.AddPoint(high2);

        aabb1.AddAxisAlignedBoundinBox(aabb2);
        mathtests::Utils::CheckValues(aabb1.min, -5.f, -2.f, -1.f);
        mathtests::Utils::CheckValues(aabb1.max, 1.f, 4.f, 3.f);
    }
}

TEST(AxisAlignedBoundingBoxUnitTests, TestAxisAlignedBoundingBoxCompute)
{
    using namespace ctrigrid;

    const Vector3 low(0.f, -2.f, 1.f);
    const Vector3 high(1.f, 2.f, 3.f);

    AxisAlignedBoundingBox aabb;
    aabb.Reset();
    aabb.AddPoint(low);
    aabb.AddPoint(high);

    Vector3 extends = aabb.ComputeExtends();
    mathtests::Utils::CheckValues(extends, 0.5f, 2.f, 1.f);

    Vector3 center = aabb.ComputeCenter();
    mathtests::Utils::CheckValues(center, 0.5f, 0.f, 2.f);
}

TEST(AxisAlignedBoundingBoxUnitTests, TestAxisAlignedBoundingBoxContains)
{
    using namespace ctrigrid;

    const Vector3 low(-1.f, -1.f, -1.f);
    const Vector3 high(1.f, 1.f, 1.f);

    AxisAlignedBoundingBox aabb;
    aabb.Reset();
    aabb.AddPoint(low);
    aabb.AddPoint(high);

    // check points out
    {
        bool r = aabb.Contains(Vector3(2.f, 0.f, 0.f)); EXPECT_FALSE(r);
        r = aabb.Contains(Vector3(-2.f, 0.f, 0.f)); EXPECT_FALSE(r);
        r = aabb.Contains(Vector3(0.f, 2.f, 0.f)); EXPECT_FALSE(r);
        r = aabb.Contains(Vector3(0.f, -2.f, 0.f)); EXPECT_FALSE(r);
        r = aabb.Contains(Vector3(0.f, 0.f, 2.f)); EXPECT_FALSE(r);
        r = aabb.Contains(Vector3(0.f, 0.f, -2.f)); EXPECT_FALSE(r);
    }

    // check point in
    {
        bool r = aabb.Contains(Vector3::ZERO);
        EXPECT_TRUE(r);
    }
}