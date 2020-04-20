
#include <gtest/gtest.h>

#include <ctrigrid/TriQueries.h>

#include "MathTestUtils.h"


// TODO: more no-overlap axis?

TEST(IntersectionQueryUnitTests, AxisAlignedBoxTriWithNoOverlap)
{
	using namespace ctrigrid;

    // setup test axis aligned box
    AxisAlignedBoundingBox bbox;
    {
        bbox.Reset();
       
        Vector3 p = Vector3::UNARY;
        bbox.AddPoint(p);
        
        p.Add(Vector3::UNARY);
        bbox.AddPoint(p);
    }


    // test x-axis separating
    {
        Vector3 v0(3.f, 1.5f, 1.45f);
        Vector3 v1(5.f, 1.5f, 1.65f);
        Vector3 v2(4.f, 0.f, 1.75f);

        bool r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2);
        EXPECT_FALSE(r);

        // move tris on the other side of the box relative to the x-axis
        const Vector3 delta(5.f, 0.f, 0.f);
        v0.Sub(delta);
        v1.Sub(delta);
        v2.Sub(delta);

        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2);
        EXPECT_FALSE(r);
    }

    // test y-axis separating
    {
        Vector3 v0(1.5f, 3.f, 1.45f);
        Vector3 v1(1.5f, 5.f, 1.65f);
        Vector3 v2(3.f, 4.f, 1.75f);

        bool r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2);
        EXPECT_FALSE(r);

        // move tris on the other side of the box relative to the x-axis
        const Vector3 delta(0.f, 5.f, 0.f);
        v0.Sub(delta);
        v1.Sub(delta);
        v2.Sub(delta);

        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2);
        EXPECT_FALSE(r);
    }

    // test z-axis separating
    {
        Vector3 v0(1.5f, 1.25f, 3.f);
        Vector3 v1(1.5f, 1.75f, 5.f);
        Vector3 v2(3.f, 1.55f, 4.f);

        bool r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2);
        EXPECT_FALSE(r);

        // move tris on the other side of the box relative to the x-axis
        const Vector3 delta(0.f, 0.f, 5.f);
        v0.Sub(delta);
        v1.Sub(delta);
        v2.Sub(delta);

        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2);
        EXPECT_FALSE(r);
    }
}

TEST(IntersectionQueryUnitTests, AxisAlignedBoxTriWithOverlap)
{
    using namespace ctrigrid;

    // setup test axis aligned box
    AxisAlignedBoundingBox bbox;
    {
        bbox.Reset();

        Vector3 p = Vector3::UNARY;
        bbox.AddPoint(p);

        p.Add(Vector3::UNARY);
        bbox.AddPoint(p);
    }


    // test overlap on +-x sides & +-xy side faces
    {
        // test tris around the bbox center 
        Vector3 v0 = bbox.ComputeCenter();
        Vector3 v1(3.f, 2.f, 1.75f);
        Vector3 v2(3.f, 1.f, 1.25f);
        Vector3 v3(2.f, 3.f, 1.55f);
        Vector3 v4(2.f, 0.f, 1.35f);

        bool r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2); EXPECT_TRUE(r);
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v3); EXPECT_TRUE(r);
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v4); EXPECT_TRUE(r);

        // switch side
        v1.x = v2.x = 0.f;
        v3.x = v4.x = 1.f;
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2); EXPECT_TRUE(r);
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v3); EXPECT_TRUE(r);
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v4); EXPECT_TRUE(r);
    }

    // test overlap on +-y side faces
    {
        // test tris around the bbox center 
        Vector3 v0 = bbox.ComputeCenter();
        Vector3 v1(2.f, 3.f, 1.75f);
        Vector3 v2(1.f, 3.f, 1.25f);

        bool r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2); EXPECT_TRUE(r);

        // switch side
        v1.y = v2.y = 0.f;
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2); EXPECT_TRUE(r);
    }

    // test overlap on +-z & +-zx side faces
    {
        // test tris around the bbox center 
        Vector3 v0 = bbox.ComputeCenter();
        Vector3 v1(2.f, 1.75f, 3.f);
        Vector3 v2(1.f, 1.25f, 3.f);
        Vector3 v3(3.f, 1.55f, 2.f);
        Vector3 v4(0.f, 1.35f, 2.f);

        bool r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2); EXPECT_TRUE(r);
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v3); EXPECT_TRUE(r);
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v4); EXPECT_TRUE(r);

        // switch side
        v1.z = v2.z = 0.f;
        v3.z = v4.z = 1.f;
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2); EXPECT_TRUE(r);
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v3); EXPECT_TRUE(r);
        r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v4); EXPECT_TRUE(r);
    }

    // test full overlaps
    {
        // tri inside the bbox
        {
            Vector3 v0(1.2f, 1.8f, 1.75f);
            Vector3 v1(1.3f, 1.2f, 1.45f);
            Vector3 v2(1.8f, 1.6f, 1.25f);
            bool r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2); EXPECT_TRUE(r);
        }

        // tri with no edge intersection with the bbox, only the interior overlaps
        {
            Vector3 v0(0.f, 3.f, 0.f);
            Vector3 v1(3.f, 0.f, -10.f);
            Vector3 v2(3.f, 0.f, 10.f);
            bool r = IntersectionQuery::OverlapAxisAlignedBoxTri(bbox, v0, v1, v2); EXPECT_TRUE(r);
        }
    }
}