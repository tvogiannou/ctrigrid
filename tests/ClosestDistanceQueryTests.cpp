
#include <gtest/gtest.h>

#include <ctrigrid/TriQueries.h>

#include "MathTestUtils.h"

// utility for returning the count of elements in a C array, type-safe
template <typename T, size_t N>
constexpr size_t countof(T const (&)[N]) noexcept { return N; }


// TODO: edge cases

struct ClosestDistQueryTestData
{
    ctrigrid::Vector3 point;
    ctrigrid::Vector3 closestPoint;
    //float dist = -1.f;
};

TEST(ClosestDistanceQueryUnitTests, ClosestPointOnTri)
{
	using namespace ctrigrid;

    // the SSE implementation of the ClosestPointOnTri uses approximate rsqrt which introduces
    // precision errors so we need to use some tolerance when comparing with expected results
    const float tolerance = 1e-2f;

    Vector3 closestPoint;

    // test acute triangle
    {
        const Vector3 v0(1.f, 0.f, 10.f);
        const Vector3 v1(0.f, 2.f, 10.f);
        const Vector3 v2(-2.f, 0.f, 10.f);

        ClosestDistQueryTestData testData[] =
        {
            // point closest to triangle plane
            { Vector3(0.f, 1.0f, 15.f), Vector3(0.f, 1.f, 10.f) },
            { Vector3(0.f, 1.0f, -10.f), Vector3(0.f, 1.f, 10.f) },

            // point closest to an edge
            { Vector3(1.f, 2.f, 10.f), Vector3(.2f, 1.6f, 10.f) },
            { Vector3(-2.f, 2.f, 10.f), Vector3(-1.f, 1.f, 10.f) },
            { Vector3(0.f, -1.f, 10.f), Vector3(0.f, 0.f, 10.f) },

            // point closest to vertex 0
            { Vector3(2.f, -0.1f, 10.f), v0 },
            { Vector3(2.f, 0.1f, 10.f), v0 },
            { Vector3(1.1f, -2.f, 10.f), v0 },

            // point closest to vertex 1
            { Vector3(0.f, 3.f, 10.f), v1 },
            { Vector3(-1.f, 3.9f, 10.f), v1 },
            { Vector3(4.f, 5.9f, 10.f), v1 },

            // point closest to vertex 2
            { Vector3(-3.f, -0.1f, 10.f), v2 },
            { Vector3(-3.f, 0.1f, 10.f), v2 },
            { Vector3(-2.1f, -2.f, 10.f), v2 },
        };

        for (uint32_t i = 0; i < countof(testData); ++i)
        {
            const ClosestDistQueryTestData& t = testData[i];

            const bool r = ClosestDistanceQuery::ClosestPointOnTri(t.point, v0, v1, v2, closestPoint);
            EXPECT_TRUE(r);
            mathtests::Utils::CheckValues(closestPoint, t.closestPoint, tolerance);
        }
    }

    // test obtuse triangle
    {
        const Vector3 v0(2.f, 0.f, 10.f);
        const Vector3 v1(0.f, 1.f, 10.f);
        const Vector3 v2(-2.f, 0.f, 10.f);

        // only test the vertex with the obtuse angle
        ClosestDistQueryTestData testData[] =
        {
            { Vector3(0.f, 3.f, 10.f), v1 },
            { Vector3(-2.f, 2.1f, 10.f), Vector3(-1.16f, 0.42f, 10.f) },
            { Vector3(2.f, 2.1f, 10.f), Vector3(1.16f, 0.42f, 10.f) },
        };

        for (uint32_t i = 0; i < countof(testData); ++i)
        {
            const ClosestDistQueryTestData& t = testData[i];

            // test all edge permutations
            bool r = ClosestDistanceQuery::ClosestPointOnTri(t.point, v0, v1, v2, closestPoint);
            EXPECT_TRUE(r);
            mathtests::Utils::CheckValues(closestPoint, t.closestPoint);

            r = ClosestDistanceQuery::ClosestPointOnTri(t.point, v2, v0, v1, closestPoint);
            EXPECT_TRUE(r);
            mathtests::Utils::CheckValues(closestPoint, t.closestPoint);

            r = ClosestDistanceQuery::ClosestPointOnTri(t.point, v1, v2, v0, closestPoint);
            EXPECT_TRUE(r);
            mathtests::Utils::CheckValues(closestPoint, t.closestPoint);
        }
    }
}

TEST(ClosestDistanceQueryUnitTests, ClosestPointOnAxisAlignedBox)
{
    using namespace ctrigrid;

    Vector3 closestPoint;

    AxisAlignedBoundingBox bbox;
    bbox.Reset();
    bbox.AddPoint(Vector3::UNARY);
    bbox.AddPoint(Vector3(3.f, 3.f, 3.f));

    const Vector3 c = bbox.ComputeCenter();
    const Vector3 e = bbox.ComputeExtends();

    {
        ClosestDistQueryTestData testData[] =
        {
            { Vector3::ZERO, bbox.min },            // near min
            { Vector3(4.f, 4.f, 4.f), bbox.max },   // near max

            { Vector3(c.x + 2.f * e.x, c.y, c.z), Vector3(c.x + e.x, c.y, c.z) },    // x+
            { Vector3(c.x - 2.f * e.x, c.y, c.z), Vector3(c.x - e.x, c.y, c.z) },    // x-

            { Vector3(c.x, c.y + 2.f * e.x, c.z), Vector3(c.x, c.y + e.x, c.z) },    // x+
            { Vector3(c.x, c.y - 2.f * e.x, c.z), Vector3(c.x, c.y - e.x, c.z) },    // x-

            { Vector3(c.x, c.y, c.z - 2.f * e.x), Vector3(c.x, c.y, c.z - e.x) },    // x-
            { Vector3(c.x, c.y, c.z + 2.f * e.x), Vector3(c.x, c.y, c.z + e.x) },    // x+
        };

        for (uint32_t i = 0; i < countof(testData); ++i)
        {
            const ClosestDistQueryTestData& t = testData[i];

            const bool r = ClosestDistanceQuery::ClosestPointOnAxisAlignedBox(t.point, bbox, closestPoint);
            EXPECT_TRUE(r);
            mathtests::Utils::CheckValues(closestPoint, t.closestPoint);
        }

    }
}