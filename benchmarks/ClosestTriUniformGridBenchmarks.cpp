
#include <benchmark/benchmark.h>

#include <ctrigrid/ClosestTriUniformGrid.h>

#include <cmath>
#include <random>
#include <vector>


const float pi = std::acos(-1.f);

static
ctrigrid::Vector3 
s_getCartesianCoords(float theta, float phi)
{
    ctrigrid::Vector3 v;

    const float sinTheta = std::sin(theta);
    v.x = sinTheta * std::cos(phi);
    v.y = sinTheta * std::sin(phi);
    v.z = std::cos(theta);

    return v;
}

static
void
s_appendToFlatVector(const ctrigrid::Vector3& v, std::vector<float>& a)
{
    a.emplace_back(v.x);
    a.emplace_back(v.y);
    a.emplace_back(v.z);
}

static
void
s_createSphereMesh(uint32_t N, uint32_t M, 
    std::vector<float>& vertices, std::vector<uint32_t>& indices)
{
    using namespace ctrigrid;

    const float thetaStep = pi / (float)N;
    const float phiStep = 2.f * pi / (float)M;

    vertices.clear();
    indices.clear();

    vertices.reserve((N - 1) * M + 2);
    indices.reserve(2 * M * (N - 1));

    // add first vertices in poles
    s_appendToFlatVector(Vector3(0.f, 0.f, 1.f), vertices);     // north pole in index [0]
    s_appendToFlatVector(Vector3(0.f, 0.f, -1.f), vertices);    // south pole in index [1]

    uint32_t lastTriIndex = 1;

    // add first longitude
    float theta = 0.f;
    for (size_t i = 0; i < N-1; ++i)
    {
        theta += thetaStep;
        s_appendToFlatVector(s_getCartesianCoords(theta, 0.f), vertices);
        ++lastTriIndex;
    }

    // add the remaining longitudes and form tris between them
    float phi = 0.f;
    for (size_t Mi = 0u; Mi < M - 1; ++Mi)
    {
        phi += phiStep;
        theta = thetaStep;

        // tri with north pole
        {
            s_appendToFlatVector(s_getCartesianCoords(theta, phi), vertices);
            ++lastTriIndex;

            const uint32_t idx = lastTriIndex;
            const uint32_t prevIdx = idx - (N - 1);

            indices.emplace_back(idx);
            indices.emplace_back(0u);
            indices.emplace_back(prevIdx);
        }

        for (size_t i = 1; i < N - 1; ++i)
        {
            theta += thetaStep;

            s_appendToFlatVector(s_getCartesianCoords(theta, phi), vertices);
            ++lastTriIndex;

            const uint32_t idx = lastTriIndex;
            const uint32_t prevIdx = idx - (N - 1);

            // tris from quad
            indices.emplace_back(idx - 1u);
            indices.emplace_back(prevIdx - 1u);
            indices.emplace_back(prevIdx);

            indices.emplace_back(idx);
            indices.emplace_back(idx - 1u);
            indices.emplace_back(prevIdx);
        }

        // tri with south pole
        {
            const uint32_t idx = lastTriIndex;
            const uint32_t prevIdx = idx - (N - 1);

            indices.emplace_back(idx);
            indices.emplace_back(prevIdx);
            indices.emplace_back(1u);
        }
    }
}



static 
void 
BM_UniformGridTriSpatialMapClosestPointQuerySinglePoint(benchmark::State& state)
{
	using namespace ctrigrid;

    const uint32_t Nsphere = (uint32_t)(std::sqrt((float)state.range(0) * .5f));
    const uint32_t Ngrid = (uint32_t)state.range(1);

    ClosestTriUniformGrid grid;
    {
        std::vector<float> vertices;
        std::vector<uint32_t> indices;
        s_createSphereMesh(Nsphere, Nsphere, vertices, indices);

        ClosestTriUniformGrid::InitInfo info;
        info.Nx = Ngrid;
        info.Ny = Ngrid;
        info.Nz = Ngrid;
        info.origin = Vector3(-2.f, -2.f, -2.f);
        info.cellWidth = 4.f / (float)Ngrid;
        grid.Init(info);

        grid.BeginGridSetup();
        grid.AddTriMesh(vertices, indices);
        grid.FinalizeGridSetup();
    }

    const Vector3 p = Vector3::UNARY;
    Vector3 cp;
    ClosestTriUniformGrid::MapTriKeyType triKey;

    for (auto _ : state)
        grid.FindClosestPointOnTris(p, cp, triKey);

    // only the size of the cell indices seems to be of importance as it highly outscales
    // the rest of the metrics
    ClosestTriUniformGrid::MemoryStats stats = grid.ComputeMemFootprint();
    //state.counters["Vertices"] = (double)stats.verticesAllocMem;
    //state.counters["Tris"] = (double)stats.trisAllocMem;
    //state.counters["Cells"] = (double)stats.cellsAllocMem;
    state.counters["Indices"] = (double)stats.cellIndicesAllocMem;
}
BENCHMARK(BM_UniformGridTriSpatialMapClosestPointQuerySinglePoint)
	->Unit(benchmark::kMicrosecond)
    ->RangeMultiplier(4)
    ->Ranges({ {2048, 150024}, {8, 32} });

static
void
BM_UniformGridTriSpatialMapClosestPointQueryMultiplePoints(benchmark::State& state)
{
    using namespace ctrigrid;

    const uint32_t Nsphere = (uint32_t)(std::sqrt((float)state.range(0) * .5f));
    const uint32_t Ngrid = 32u;
    
    // generate grid
    ClosestTriUniformGrid grid;
    {
        std::vector<float> vertices;
        std::vector<uint32_t> indices;
        s_createSphereMesh(Nsphere, Nsphere, vertices, indices);

        ClosestTriUniformGrid::InitInfo info;
        info.Nx = Ngrid;
        info.Ny = Ngrid;
        info.Nz = Ngrid;
        info.origin = Vector3(-2.f, -2.f, -2.f);
        info.cellWidth = 4.f / (float)Ngrid;
        grid.Init(info);

        grid.BeginGridSetup();
        grid.AddTriMesh(vertices, indices);
        grid.FinalizeGridSetup();
    }

    // generate random points
    std::vector<Vector3> points;
    {
        const float tolerance = 1e-5f; 
        
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        gen.seed(1234);

        const AxisAlignedBoundingBox& bbox = grid.GetGridAABoxWorldSpace();
        std::uniform_real_distribution<float> disX(bbox.min.x + tolerance, bbox.max.x - tolerance);
        std::uniform_real_distribution<float> disY(bbox.min.y + tolerance, bbox.max.y - tolerance);
        std::uniform_real_distribution<float> disZ(bbox.min.z + tolerance, bbox.max.z - tolerance);

        const uint32_t N = (uint32_t)state.range(1);

        for (uint32_t i = 0; i < N; ++i)
        {
            Vector3 p(disX(gen), disY(gen), disZ(gen));
            points.push_back(p);
        }
    }

    Vector3 cp;
    ClosestTriUniformGrid::MapTriKeyType triKey;

    for (auto _ : state)
    {
        for (const Vector3& v : points)
            grid.FindClosestPointOnTris(v, cp, triKey);
    }

}
BENCHMARK(BM_UniformGridTriSpatialMapClosestPointQueryMultiplePoints)
    ->Unit(benchmark::kMillisecond)
    ->RangeMultiplier(4)
    ->Ranges({ {2048, 150024}, {128, 50000} });
