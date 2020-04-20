
#include <benchmark/benchmark.h>

#include <ctrigrid/Compiler.h>
#include <ctrigrid/Vector3.h>
#include <ctrigrid/TriQueries.h>

template <typename T, size_t N>
constexpr size_t countof(T const (&)[N]) noexcept { return N; }

static
void
BM_ClosestPointOnTriAllCases(benchmark::State& state)
{
    using namespace ctrigrid;

    Vector3 testPoints[] =
    {
        // point closest to triangle plane
        Vector3(0.f, 1.0f, 15.f), 
        Vector3(0.f, 1.0f, -10.f), 

        // point closest to an edge
        Vector3(1.f, 2.f, 10.f), 
        Vector3(-2.f, 2.f, 10.f), 
        Vector3(0.f, -1.f, 10.f), 

        // point closest to vertex 0
        Vector3(2.f, -0.1f, 10.f), 
        Vector3(2.f, 0.1f, 10.f), 
        Vector3(1.1f, -2.f, 10.f), 

        // point closest to vertex 1
        Vector3(0.f, 3.f, 10.f), 
        Vector3(-1.f, 3.9f, 10.f), 
        Vector3(4.f, 5.9f, 10.f), 

        // point closest to vertex 2
        Vector3(-3.f, -0.1f, 10.f), 
        Vector3(-3.f, 0.1f, 10.f),
        Vector3(-2.1f, -2.f, 10.f),
    };

    // acute triangle
    const Vector3 acV0(1.f, 0.f, 10.f);
    const Vector3 acV1(0.f, 2.f, 10.f);
    const Vector3 acV2(-2.f, 0.f, 10.f);

    // obtuse triangle
    const Vector3 obtV0(2.f, 0.f, 10.f);
    const Vector3 obtV1(0.f, 1.f, 10.f);
    const Vector3 obtV2(-2.f, 0.f, 10.f);

    Vector3 cp1[countof(testPoints)];
    Vector3 cp2[countof(testPoints)];

    for (auto _ : state)
    {
        for (uint32_t i = 0u; i < countof(testPoints); ++i)
        {
            ClosestDistanceQuery::ClosestPointOnTri(testPoints[i], acV0, acV1, acV2, cp1[i]);
            ClosestDistanceQuery::ClosestPointOnTri(testPoints[i], obtV0, obtV1, acV2, cp2[i]);
        }
    }

}
BENCHMARK(BM_ClosestPointOnTriAllCases)->Unit(benchmark::kNanosecond);
    