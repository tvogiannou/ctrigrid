
#include <gtest/gtest.h>

#include <ctrigrid/ClosestTriUniformGrid.h>
#include <ctrigrid/TriQueries.h>

#include <vector>
#include <random>
#include <functional>


using BuilderStatus = ctrigrid::ClosestTriUniformGrid::Builder::BuilderStatus;


static 
void s_CheckStoredTrisInCell(
    const ctrigrid::ClosestTriUniformGrid::CellIndex3& Nxyz,
    ctrigrid::ClosestTriUniformGrid::CellIndex i, 
    ctrigrid::ClosestTriUniformGrid::CellIndex j, 
    ctrigrid::ClosestTriUniformGrid::CellIndex k,
    size_t expectedCount,
    std::function<          // function to retrieve the triangles to test againt
    BuilderStatus           // it is different between the grid and the builder
        (ctrigrid::ClosestTriUniformGrid::CellKey key,              
         ctrigrid::ClosestTriUniformGrid::TriKeyArray&)> GetTris)
{
    using namespace ctrigrid;
   
    ClosestTriUniformGrid::CellKey key;

    bool r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(
                Nxyz, ClosestTriUniformGrid::ToIndex3(i, j, k), key);
    EXPECT_TRUE(r);

    ClosestTriUniformGrid::TriKeyArray tris;
    BuilderStatus s = GetTris(key, tris);    
    EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);
    
    EXPECT_EQ(tris.size(), expectedCount);
}

// #define CTRIGRID_TEST_GRID_PRINT_OUT
#ifdef CTRIGRID_TEST_GRID_PRINT_OUT 
static
void s_PrintGridZSlice(
    const ctrigrid::ClosestTriUniformGrid &grid, 
    ctrigrid::ClosestTriUniformGrid::CellIndex k)
{
    using namespace ctrigrid;

    const ClosestTriUniformGrid::CellIndex3 Nxyz = 
        ClosestTriUniformGrid::ToIndex3(
            grid.GetNumberCellsXAxis(),
            grid.GetNumberCellsYAxis(),
            grid.GetNumberCellsZAxis());

    for (ClosestTriUniformGrid::CellIndex i = 0u; i < grid.GetNumberCellsXAxis(); ++i)
    {
        for (int64_t j = grid.GetNumberCellsYAxis() - 1u; j >= 0u; --j)
        {
            const ClosestTriUniformGrid::CellIndex3 ijk = 
                ClosestTriUniformGrid::ToIndex3(i, (ClosestTriUniformGrid::CellIndex)j, k);

            ClosestTriUniformGrid::CellKey cellKey;
            ClosestTriUniformGrid::ComputeCellKeyFromIndex(Nxyz, ijk, cellKey);

            std::cout << "(";
            ClosestTriUniformGrid::TriKeyArray tris;
            grid.GetClosestTrisOnCell(cellKey, tris);
            for (ClosestTriUniformGrid::TriKey t : tris)
                std::cout << " " << t;
            std::cout << ")\t";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}
#endif


TEST(ClosestTriUniformGridUnitTests, UniformGridTriIndexing)
{
    using namespace ctrigrid;

    // test index -> key and key -> index conversions

    // non initialized should fail
    {
        ClosestTriUniformGrid::CellKey key;
        bool r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(
            ClosestTriUniformGrid::ToIndex3(0u, 0u, 0u), ClosestTriUniformGrid::ToIndex3(0u, 0u, 0u), key);
        EXPECT_FALSE(r);

        ClosestTriUniformGrid::CellIndex i, j, k;
        r = ClosestTriUniformGrid::ComputeIndexFromCellKey(
            ClosestTriUniformGrid::ToIndex3(0u, 0u, 0u), key, i, j, k);
        EXPECT_FALSE(r);
    }

    // even sizes
    {
        constexpr ClosestTriUniformGrid::CellKey Nx = 12u;
        constexpr ClosestTriUniformGrid::CellKey Ny = 8u;
        constexpr ClosestTriUniformGrid::CellKey Nz = 4u;
        const ClosestTriUniformGrid::CellIndex3 N = ClosestTriUniformGrid::ToIndex3(Nx, Ny, Nz);

        ClosestTriUniformGrid::CellKey key;
        for (ClosestTriUniformGrid::CellIndex i = 0u; i < Nx; ++i)
        {
            for (ClosestTriUniformGrid::CellIndex j = 0u; j < Ny; ++j)
            {
                for (ClosestTriUniformGrid::CellIndex k = 0u; k < Nz; ++k)
                {
                    const ClosestTriUniformGrid::CellIndex3 ijk = 
                                ClosestTriUniformGrid::ToIndex3(i, j, k);

                    bool r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(N, ijk, key);
                    EXPECT_TRUE(r);

                    ClosestTriUniformGrid::CellIndex i_, j_, k_;
                    r = ClosestTriUniformGrid::ComputeIndexFromCellKey(N, key, i_, j_, k_);
                    EXPECT_TRUE(r);
                    EXPECT_EQ(i, i_);
                    EXPECT_EQ(j, j_);
                    EXPECT_EQ(k, k_);
                }
            }
        }

        // cases that should fail
        bool r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(
            N, ClosestTriUniformGrid::ToIndex3(Nx, 0u, 0u), key); EXPECT_FALSE(r);
        r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(
            N, ClosestTriUniformGrid::ToIndex3(0u, Ny, 0u), key); EXPECT_FALSE(r);
        r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(
            N, ClosestTriUniformGrid::ToIndex3(0u, 0u, Nz), key); EXPECT_FALSE(r);
    }

    // odd sizes
    {
        const ClosestTriUniformGrid::CellKey Nx = 13u;
        const ClosestTriUniformGrid::CellKey Ny = 7u;
        const ClosestTriUniformGrid::CellKey Nz = 5u;
        const ClosestTriUniformGrid::CellIndex3 N = ClosestTriUniformGrid::ToIndex3(Nx, Ny, Nz);

        ClosestTriUniformGrid::CellKey key;
        for (ClosestTriUniformGrid::CellIndex i = 0u; i < Nx; ++i)
        {
            for (ClosestTriUniformGrid::CellIndex j = 0u; j < Ny; ++j)
            {
                for (ClosestTriUniformGrid::CellIndex k = 0u; k < Nz; ++k)
                {
                    const ClosestTriUniformGrid::CellIndex3 ijk =
                                ClosestTriUniformGrid::ToIndex3(i, j, k);
                    

                    bool r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(N, ijk, key);
                    EXPECT_TRUE(r);

                    ClosestTriUniformGrid::CellIndex i_, j_, k_;
                    r = ClosestTriUniformGrid::ComputeIndexFromCellKey(N, key, i_, j_, k_);
                    EXPECT_TRUE(r);
                    EXPECT_EQ(i, i_);
                    EXPECT_EQ(j, j_);
                    EXPECT_EQ(k, k_);
                }
            }
        }

        // cases that should fail
        bool r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(
            N, ClosestTriUniformGrid::ToIndex3(Nx, 0u, 0u), key); EXPECT_FALSE(r);
        r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(
            N, ClosestTriUniformGrid::ToIndex3(0u, Ny, 0u), key); EXPECT_FALSE(r);
        r = ClosestTriUniformGrid::ComputeCellKeyFromIndex(
            N, ClosestTriUniformGrid::ToIndex3(0u, 0u, Nz), key); EXPECT_FALSE(r);
    }
}

TEST(ClosestTriUniformGridUnitTests, UniformGridTriCellKey)
{
    using namespace ctrigrid;

    // test grid properties
    const ClosestTriUniformGrid::CellKey Nx = 4u;
    const ClosestTriUniformGrid::CellKey Ny = 4u;
    const ClosestTriUniformGrid::CellKey Nz = 4u;
    const float cellWidth = 2.f;
    const Vector3 origin = Vector3::UNARY;

    // setup a builder
    ClosestTriUniformGrid::Builder builder;
    {
        // non initialized should fail
        AxisAlignedBoundingBox bbox;
        BuilderStatus s = builder.ComputeCelBBoxWorldSpace(0u, 0u, 0u, bbox);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_INVALID_INDEX);

        ClosestTriUniformGrid::Builder::InitInfo info = { Nx, Ny, Nz, origin, cellWidth };
        
        s = builder.Init(info);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);
    }

    // test points in & out of the grid
    {
        const Vector3 pointsInside[] =
        {
            Vector3(2.f, 2.f, 2.f),
            Vector3(8.f, 8.f, 8.f)
        };

        ClosestTriUniformGrid::CellKey key;
        bool r = ClosestTriUniformGrid::ComputeCellKeyFromPoint(
            builder.Nxyz, cellWidth, builder.gridBox, pointsInside[0], key);
        EXPECT_TRUE(r);
        EXPECT_EQ(key, 0u);

        r = ClosestTriUniformGrid::ComputeCellKeyFromPoint(
            builder.Nxyz, cellWidth, builder.gridBox, pointsInside[1], key); 
        EXPECT_TRUE(r);
        EXPECT_EQ(key, Nx * Ny * Nz - 1u);

        // check points that should fail
        const Vector3 pointsOutside[] =
        {
            Vector3(0.f, 0.f, 0.f),
            Vector3(10.f, 10.f, 10.f)
        };

        r = ClosestTriUniformGrid::ComputeCellKeyFromPoint(
            builder.Nxyz, cellWidth, builder.gridBox, pointsOutside[0], key); 
        EXPECT_FALSE(r);
        
        r = ClosestTriUniformGrid::ComputeCellKeyFromPoint(
            builder.Nxyz, cellWidth, builder.gridBox, pointsOutside[1], key); 
        EXPECT_FALSE(r);
    }

    // test cell aabox
    {
        AxisAlignedBoundingBox bbox;
        BuilderStatus s = builder.ComputeCelBBoxWorldSpace(0u, 0u, 0u, bbox); 
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);
        EXPECT_FLOAT_EQ(bbox.min.x, origin.x);
        EXPECT_FLOAT_EQ(bbox.min.y, origin.y);
        EXPECT_FLOAT_EQ(bbox.min.z, origin.z);
        EXPECT_FLOAT_EQ(bbox.max.x, origin.x + cellWidth);
        EXPECT_FLOAT_EQ(bbox.max.y, origin.y + cellWidth);
        EXPECT_FLOAT_EQ(bbox.max.z, origin.z + cellWidth);

        s = builder.ComputeCelBBoxWorldSpace(Nx - 1u, Ny - 1u, Nz - 1u, bbox); 
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);
        EXPECT_FLOAT_EQ(bbox.min.x, origin.x + (Nx - 1u) * cellWidth);
        EXPECT_FLOAT_EQ(bbox.min.y, origin.y + (Ny - 1u) * cellWidth);
        EXPECT_FLOAT_EQ(bbox.min.z, origin.z + (Nz - 1u) * cellWidth);
        EXPECT_FLOAT_EQ(bbox.max.x, origin.x + Nx * cellWidth);
        EXPECT_FLOAT_EQ(bbox.max.y, origin.y + Ny * cellWidth);
        EXPECT_FLOAT_EQ(bbox.max.z, origin.z + Nz * cellWidth);


        // should fail
        s = builder.ComputeCelBBoxWorldSpace(Nx, 0u, 0u, bbox); 
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_INVALID_INDEX);
        s = builder.ComputeCelBBoxWorldSpace(0u, Ny, 0u, bbox);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_INVALID_INDEX);
        s = builder.ComputeCelBBoxWorldSpace(0u, 0u, Nz, bbox);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_INVALID_INDEX);
    }
}

TEST(ClosestTriUniformGridUnitTests, UniformGridTriBuilder)
{
    using namespace ctrigrid;


    // grid params
    const ClosestTriUniformGrid::CellKey Nx = 5u;
    const ClosestTriUniformGrid::CellKey Ny = 5u;
    const ClosestTriUniformGrid::CellKey Nz = 5u;
    const float cellWidth = 2.f;
    const Vector3 origin = Vector3::UNARY;

    // test data
    std::vector<float> vertices = 
    {
        2.5f, 6.f, 6.f,
        4.5f, 4.f, 6.f,
        6.f, 8.f, 6.f,
        6.f, 4.f, 6.f
    };
    std::vector<uint32_t> indices =
    {
        0, 1, 2,
        1, 3, 2
    };

    // create grid and add tris
    {
        ClosestTriUniformGrid::Builder builder;
        ClosestTriUniformGrid::Builder::InitInfo info = { Nx, Ny, Nz, origin, cellWidth };
        BuilderStatus s = builder.Init(info);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.BeginGridSetup();
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.AddTriMesh(vertices, indices);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        // should not finalize for this test!

        // check grid
        {
            const ClosestTriUniformGrid::CellIndex3 Nxyz = builder.Nxyz;

            auto f = std::bind(
                &ClosestTriUniformGrid::Builder::GetOverlappingTrisOnCell, 
                builder, 
                std::placeholders::_1, 
                std::placeholders::_2);

            s_CheckStoredTrisInCell(Nxyz, 0u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 0u, 1u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 0u, 2u, 2u, 1u, f);
            s_CheckStoredTrisInCell(Nxyz, 0u, 3u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 0u, 4u, 2u, 0u, f);

            s_CheckStoredTrisInCell(Nxyz, 1u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 1u, 1u, 2u, 2u, f);
            s_CheckStoredTrisInCell(Nxyz, 1u, 2u, 2u, 2u, f);
            s_CheckStoredTrisInCell(Nxyz, 1u, 3u, 2u, 1u, f);
            s_CheckStoredTrisInCell(Nxyz, 1u, 4u, 2u, 0u, f);

            s_CheckStoredTrisInCell(Nxyz, 2u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 2u, 1u, 2u, 1u, f);
            s_CheckStoredTrisInCell(Nxyz, 2u, 2u, 2u, 2u, f);
            s_CheckStoredTrisInCell(Nxyz, 2u, 3u, 2u, 2u, f);
            s_CheckStoredTrisInCell(Nxyz, 2u, 4u, 2u, 0u, f);

            s_CheckStoredTrisInCell(Nxyz, 3u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 3u, 1u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 3u, 2u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 3u, 3u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 3u, 4u, 2u, 0u, f);

            s_CheckStoredTrisInCell(Nxyz, 4u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 4u, 1u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 4u, 2u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 4u, 3u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 4u, 4u, 2u, 0u, f);
        }
    }

    // test raw array API
    {
        ClosestTriUniformGrid::Builder builder;
        ClosestTriUniformGrid::Builder::InitInfo info = { Nx, Ny, Nz, origin, cellWidth };
        BuilderStatus s = builder.Init(info);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.BeginGridSetup();
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.AddTriMesh( vertices.data(), vertices.size(), 
                                indices.data(), indices.size());
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        // should not finalize for this test!
        
        // check grid
        {
            const ClosestTriUniformGrid::CellIndex3 Nxyz = builder.Nxyz;

            auto f = std::bind(
                &ClosestTriUniformGrid::Builder::GetOverlappingTrisOnCell, 
                builder, 
                std::placeholders::_1, 
                std::placeholders::_2);

            s_CheckStoredTrisInCell(Nxyz, 0u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 0u, 1u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 0u, 2u, 2u, 1u, f);
            s_CheckStoredTrisInCell(Nxyz, 0u, 3u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 0u, 4u, 2u, 0u, f);

            s_CheckStoredTrisInCell(Nxyz, 1u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 1u, 1u, 2u, 2u, f);
            s_CheckStoredTrisInCell(Nxyz, 1u, 2u, 2u, 2u, f);
            s_CheckStoredTrisInCell(Nxyz, 1u, 3u, 2u, 1u, f);
            s_CheckStoredTrisInCell(Nxyz, 1u, 4u, 2u, 0u, f);

            s_CheckStoredTrisInCell(Nxyz, 2u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 2u, 1u, 2u, 1u, f);
            s_CheckStoredTrisInCell(Nxyz, 2u, 2u, 2u, 2u, f);
            s_CheckStoredTrisInCell(Nxyz, 2u, 3u, 2u, 2u, f);
            s_CheckStoredTrisInCell(Nxyz, 2u, 4u, 2u, 0u, f);

            s_CheckStoredTrisInCell(Nxyz, 3u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 3u, 1u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 3u, 2u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 3u, 3u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 3u, 4u, 2u, 0u, f);

            s_CheckStoredTrisInCell(Nxyz, 4u, 0u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 4u, 1u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 4u, 2u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 4u, 3u, 2u, 0u, f);
            s_CheckStoredTrisInCell(Nxyz, 4u, 4u, 2u, 0u, f);
        }
    }
    
    // TODO: test failure cases
}

TEST(ClosestTriUniformGridUnitTests, UniformGridTriBuilderFinalize)
{
    using namespace ctrigrid;

    ClosestTriUniformGrid grid;

    // grid params
    const ClosestTriUniformGrid::CellKey Nx = 5u;
    const ClosestTriUniformGrid::CellKey Ny = 5u;
    const ClosestTriUniformGrid::CellKey Nz = 5u;
    const float cellWidth = 2.f;
    const Vector3 origin = Vector3::UNARY;

    // test data
    std::vector<float> vertices =
    {
        1.25f, 9.1f, 2.f,
        2.75f, 9.2f, 2.f,
        2.f, 10.9f, 2.f,
        8.1f, 2.f, 10.f,
        10.f, 2.f, 10.f,
        10.f, 3.9f, 10.f
    };
    std::vector<uint32_t> indices =
    {
        0, 1, 2,
        3, 4, 5
    };

    // create grid and add tris
    {
        ClosestTriUniformGrid::Builder::InitInfo info = { Nx, Ny, Nz, origin, cellWidth };
        ClosestTriUniformGrid::Builder builder;
        BuilderStatus s = builder.Init(info);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.BeginGridSetup();
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.AddTriMesh(vertices, indices);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.FinalizeGridSetup(grid);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);
    }

    // check stored tris
    {
#ifdef CTRIGRID_TEST_GRID_PRINT_OUT
        // XXX print for testing
        s_PrintGridZSlice(grid, 0u);
        s_PrintGridZSlice(grid, 1u);
        s_PrintGridZSlice(grid, 2u);
        s_PrintGridZSlice(grid, 3u);
        s_PrintGridZSlice(grid, 4u);
#endif

        // just sample check some rows in the grid
        const ClosestTriUniformGrid::CellIndex3 Nxyz = ClosestTriUniformGrid::ToIndex3(Nx, Ny, Nz);
        auto f = std::bind(
            &ClosestTriUniformGrid::GetClosestTrisOnCell, 
            &grid, 
            std::placeholders::_1, 
            std::placeholders::_2);
        
        s_CheckStoredTrisInCell(Nxyz, 0u, 0u, 0u, 2u, f);
        s_CheckStoredTrisInCell(Nxyz, 0u, 1u, 0u, 2u, f);
        s_CheckStoredTrisInCell(Nxyz, 0u, 2u, 0u, 1u, f);
        s_CheckStoredTrisInCell(Nxyz, 0u, 3u, 0u, 1u, f);
        s_CheckStoredTrisInCell(Nxyz, 0u, 4u, 0u, 1u, f);

        s_CheckStoredTrisInCell(Nxyz, 0u, 1u, 4u, 2u, f);
        s_CheckStoredTrisInCell(Nxyz, 1u, 1u, 4u, 1u, f);
        s_CheckStoredTrisInCell(Nxyz, 2u, 1u, 4u, 1u, f);
        s_CheckStoredTrisInCell(Nxyz, 3u, 1u, 4u, 1u, f);
        s_CheckStoredTrisInCell(Nxyz, 4u, 1u, 4u, 1u, f);
    }
}

TEST(ClosestTriUniformGridUnitTests, UniformGridTriClosestPoint)
{
    using namespace ctrigrid;

    ClosestTriUniformGrid grid;

    const float tolerance = 1.e3f;  // use since the sse version of closest point-tri query

    // grid params
    const ClosestTriUniformGrid::CellKey Nx = 5u;
    const ClosestTriUniformGrid::CellKey Ny = 5u;
    const ClosestTriUniformGrid::CellKey Nz = 5u;
    const float cellWidth = 2.f;
    const Vector3 origin = Vector3::UNARY;

    // test data
    std::vector<float> vertices =
    {
        1.25f, 9.1f, 2.f,
        2.75f, 9.2f, 2.f,
        2.f, 10.9f, 2.f,
        8.1f, 2.f, 10.f,
        10.f, 2.f, 10.f,
        10.f, 3.9f, 10.f
    };
    std::vector<uint32_t> indices =
    {
        0, 1, 2,
        3, 4, 5
    };

    // create grid and add tris
    {
        ClosestTriUniformGrid::Builder::InitInfo info = { Nx, Ny, Nz, origin, cellWidth };
        ClosestTriUniformGrid::Builder builder;
        BuilderStatus s = builder.Init(info);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.BeginGridSetup();
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.AddTriMesh(vertices, indices);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.FinalizeGridSetup(grid);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);
    }

    // query closest points
    {
        const Vector3 p0(4.f, 10.f, 2.f);
        const Vector3 p1(2.f, 10.f, 6.f);
        const Vector3 p2(6.f, 6.f, 6.f);

        Vector3 closestPoint;
        ClosestTriUniformGrid::TriKey triKey;

        bool r = grid.FindClosestPointOnTris(p0, closestPoint, triKey); EXPECT_TRUE(r);
        EXPECT_EQ(triKey, 0u);

        r = grid.FindClosestPointOnTris(p1, closestPoint, triKey); EXPECT_TRUE(r);
        EXPECT_EQ(triKey, 0u);
        EXPECT_NEAR(closestPoint.x, p1.x, tolerance);
        EXPECT_NEAR(closestPoint.y, p1.y, tolerance);
        EXPECT_NEAR(closestPoint.z, 2.f, tolerance);

        r = grid.FindClosestPointOnTris(p2, closestPoint, triKey); EXPECT_TRUE(r);
        EXPECT_EQ(triKey, 1u);
    }

    // point outside grid cases
    {
        const Vector3 p0(11.5f, 11.5f, 11.5f);

        Vector3 closestPoint;
        ClosestTriUniformGrid::TriKey triKey;

        // by default, points outside the grid are being ignored
        bool r = grid.FindClosestPointOnTris(Vector3::ZERO, closestPoint, triKey);
        EXPECT_FALSE(r);

        r = grid.FindClosestPointOnTris(p0, closestPoint, triKey);
        EXPECT_FALSE(r);

        // relax the requirement for points outside the grid
        bool forceInGrid = false;
        r = grid.FindClosestPointOnTris(Vector3::ZERO, closestPoint, triKey, forceInGrid);
        EXPECT_TRUE(r);

        r = grid.FindClosestPointOnTris(p0, closestPoint, triKey, forceInGrid);
        EXPECT_TRUE(r);
    }
}

TEST(ClosestTriUniformGridUnitTests, UniformGridTriClosestPointRandom)
{
    using namespace ctrigrid;


    const float tolerance = 4e-2f;      // the SSE version of ClosestPointOnTri seems to introduce
                                        // significant precision errors, need to investigate
    //const float tolerance = 1e-5f;    // the non SSE version has good enough precision for this

    // create 5tris on a plane defined by the following plane vectors
    Vector3 u0(1.f, 1.f, 0.f); u0.Normalize();
    Vector3 u1(0.f, 1.f, 1.f); u1.Normalize();
    Vector3 c(-2.f, -2.f, -2.f);
    
    std::vector<float> vertices;
    {
        vertices.push_back(c.x);
        vertices.push_back(c.y);
        vertices.push_back(c.z);

        for (uint32_t i = 1u; i < 4u; ++i)
        {
            const float s = (float)i;
            vertices.push_back(c.x + s * u0.x);
            vertices.push_back(c.y + s * u0.y);
            vertices.push_back(c.z + s * u0.z);
        }
        for (uint32_t i = 1u; i < 4u; ++i)
        {
            const float s = (float)i;
            vertices.push_back(c.x + s * u1.x);
            vertices.push_back(c.y + s * u1.y);
            vertices.push_back(c.z + s * u1.z);
        }
    }

    std::vector<uint32_t> indices =
    {
        0u, 1u, 4u,
        1u, 2u, 4u,
        2u, 5u, 4u,
        2u, 6u, 5u,
        2u, 3u, 6u
    };


    ClosestTriUniformGrid grid;

    // grid params
    const ClosestTriUniformGrid::CellKey Nx = 8u;
    const ClosestTriUniformGrid::CellKey Ny = 8u;
    const ClosestTriUniformGrid::CellKey Nz = 8u;
    const float cellWidth = 1.f;
    Vector3 origin = c;
    origin.Sub(Vector3::UNARY);
    origin.Sub(Vector3::UNARY);

    // create grid and add tris
    {
        ClosestTriUniformGrid::Builder::InitInfo info = { Nx, Ny, Nz, origin, cellWidth };
        ClosestTriUniformGrid::Builder builder;
        BuilderStatus s = builder.Init(info);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.BeginGridSetup();
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.AddTriMesh(vertices, indices);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);

        s = builder.FinalizeGridSetup(grid);
        EXPECT_TRUE(s == BuilderStatus::eBUILDER_STATUS_SUCCESS);
    }

    // generate random points and respective closest points for testing
    std::vector<Vector3> points;
    std::vector<Vector3> closestPoints;
    {
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        gen.seed(1234);

        const AxisAlignedBoundingBox& bbox = grid.GetGridAABoxWorldSpace();
        std::uniform_real_distribution<float> disX(bbox.min.x + tolerance, bbox.max.x - tolerance);
        std::uniform_real_distribution<float> disY(bbox.min.y + tolerance, bbox.max.y - tolerance);
        std::uniform_real_distribution<float> disZ(bbox.min.z + tolerance, bbox.max.z - tolerance);

        const uint32_t N = 512u;
        
        for (uint32_t i = 0; i < N; ++i)
        {
            Vector3 p(disX(gen), disY(gen), disZ(gen));
            points.push_back(p);
        }

        // find closest points with brute force
        for (const Vector3& p : points)
        {
            Vector3 cp = Vector3::ZERO;
            float minDist = 10000.f * cellWidth;
            for (size_t j = 0; j < indices.size(); j += 3)
            {
                const size_t triIdx0 = indices[j];
                const size_t triIdx1 = indices[j + 1];
                const size_t triIdx2 = indices[j + 2];

                const Vector3 v0 = Vector3(vertices[3 * triIdx0], vertices[3 * triIdx0 + 1], vertices[3 * triIdx0 + 2]);
                const Vector3 v1 = Vector3(vertices[3 * triIdx1], vertices[3 * triIdx1 + 1], vertices[3 * triIdx1 + 2]);
                const Vector3 v2 = Vector3(vertices[3 * triIdx2], vertices[3 * triIdx2 + 1], vertices[3 * triIdx2 + 2]);

                Vector3 temp;
                ClosestDistanceQuery::ClosestPointOnTri(p, v0, v1, v2, temp);

                Vector3 d = p;
                d.Sub(temp);
                const float dist = d.Dot(d);
                if (dist < minDist)
                {
                    cp = temp;
                    minDist = dist;
                }
            }
            closestPoints.push_back(cp);
        }
    }

    // test all random points
    for (size_t i = 0; i < points.size(); i++)
    {
        Vector3 cp;
        ClosestTriUniformGrid::TriKey triKey;
        bool r = grid.FindClosestPointOnTris(points[i], cp, triKey);

        const Vector3& expectedCP = closestPoints[i];
        EXPECT_TRUE(r);
        EXPECT_NEAR(cp.x, expectedCP.x, tolerance);
        EXPECT_NEAR(cp.y, expectedCP.y, tolerance);
        EXPECT_NEAR(cp.z, expectedCP.z, tolerance);
    }
}    