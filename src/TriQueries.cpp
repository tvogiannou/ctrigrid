
#include <ctrigrid/TriQueries.h>

#include <ctrigrid/MathOptConfig.h>

#include <algorithm>



#ifdef CTRIGRID_TRIPOINTDISTQUERY_SSE
    #include <ctrigrid/Vector4.h>

    // helpers to hide verbose cast intrinsics
    #define CTRIGRID_MATH_SSE_SHIFT_RIGHT(vec, b) _mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(vec), b))
    #define CTRIGRID_MATH_SSE_SHIFT_LEFT(vec, b) _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(vec), b))
#endif


namespace ctrigrid
{

bool s_OriginAABoxTriAxisOverlap(
    const Vector3& extends,
    const Vector3& axis,
    const Vector3& v0, const Vector3& v1, const Vector3& v2)
{
    const float p0 = v0.Dot(axis);
    const float p1 = v1.Dot(axis);
    const float p2 = v2.Dot(axis);

    // r = sum{extends * axis}
    Vector3 n = axis;
    n.Abs();
    Vector3 rv = extends;
    rv.Mul(n);
    const float r = rv.Sum();

    // compare min & max values
    const auto p = std::minmax({ p0, p1, p2 });
    if (p.first > r || p.second < -r)
        return false;

    return true;
}

bool 
IntersectionQuery::OverlapAxisAlignedBoxTri(
    const AxisAlignedBoundingBox& aabox, 
    const Vector3& triV0, const Vector3& triV1, const Vector3& triV2)
{
    const Vector3 c = aabox.ComputeCenter();
    const Vector3 extends = aabox.ComputeExtends();

    // translate tri so that aabox is at the origin
    Vector3 v0 = triV0; v0.Sub(c);
    Vector3 v1 = triV1; v1.Sub(c);
    Vector3 v2 = triV2; v2.Sub(c);

    // X, Y, Z axis tests
    {
        const auto px = std::minmax({ v0.x, v1.x, v2.x });
        const auto py = std::minmax({ v0.y, v1.y, v2.y });
        const auto pz = std::minmax({ v0.z, v1.z, v2.z });

        if (px.first > extends.x || px.second < -extends.x) return false;
        if (py.first > extends.y || py.second < -extends.y) return false;
        if (pz.first > extends.z || pz.second < -extends.z) return false;
    }

    Vector3 e0 = v1; e0.Sub(v0);    // e0 = v1 - v0
    Vector3 e1 = v2; e1.Sub(v1);    // e1 = v2 - v1
    Vector3 e2 = v0; e2.Sub(v2);    // e2 = v0 - v2

    Vector3 n = Vector3::Cross(e0, e1);

    // test axis -> tri normal
    if (!s_OriginAABoxTriAxisOverlap(extends, n, v0, v1, v2)) return false;

    // test axis -> edge cross product with X-axis
    if (!s_OriginAABoxTriAxisOverlap(extends, Vector3(0.f, -e0.z, e0.y), v0, v1, v2)) return false;
    if (!s_OriginAABoxTriAxisOverlap(extends, Vector3(0.f, -e1.z, e1.y), v0, v1, v2)) return false;
    if (!s_OriginAABoxTriAxisOverlap(extends, Vector3(0.f, -e2.z, e2.y), v0, v1, v2)) return false;

    // test axis -> edge cross product with Y-axis
    if (!s_OriginAABoxTriAxisOverlap(extends, Vector3(e0.z, 0.f, -e0.x), v0, v1, v2)) return false;
    if (!s_OriginAABoxTriAxisOverlap(extends, Vector3(e1.z, 0.f, -e1.x), v0, v1, v2)) return false;
    if (!s_OriginAABoxTriAxisOverlap(extends, Vector3(e2.z, 0.f, -e2.x), v0, v1, v2)) return false;

    // test axis -> edge cross product with Z-axis
    if (!s_OriginAABoxTriAxisOverlap(extends, Vector3(-e0.y, e0.x, 0.f), v0, v1, v2)) return false;
    if (!s_OriginAABoxTriAxisOverlap(extends, Vector3(-e1.y, e1.x, 0.f), v0, v1, v2)) return false;
    if (!s_OriginAABoxTriAxisOverlap(extends, Vector3(-e2.y, e2.x, 0.f), v0, v1, v2)) return false;

    return true;
}


enum PointLineSegmentHint : int
{
    eCLOSEST_POINT_ON_SEGMENT = 0,
    eCLOSEST_POINT_ON_START,
    eCLOSEST_POINT_ON_END
};

static
PointLineSegmentHint s_ClosestPointLineSegment(
    const Vector3& dPointVstart,    // P - Vstart
    const Vector3& e,               // edge vector, i.e. Vend - Vstart
    const Vector3& Vstart, const Vector3& Vend,
    Vector3& closestPoint)
{
    PointLineSegmentHint hint = PointLineSegmentHint::eCLOSEST_POINT_ON_SEGMENT;

    const float l = dPointVstart.Dot(e);    // project to non-normalized edge
    const float s = e.Dot(e);               // compare with square 

    if (l > 0.f && l < s)
    {
        // C = Vstart + l * e
        closestPoint = Vstart;
        closestPoint.MulAdd(l / s, e);
    }
    else
    {
        if (l <= 0.f)
        {
            closestPoint = Vstart;
            hint = PointLineSegmentHint::eCLOSEST_POINT_ON_START;
        }
        else //if (l >= s)
        {
            closestPoint = Vend;
            hint = PointLineSegmentHint::eCLOSEST_POINT_ON_END;
        }
    }

    return hint;
}



bool ClosestDistanceQuery::ClosestPointOnTri(
    const Vector3& point,
    const Vector3& triV0, const Vector3& triV1, const Vector3& triV2,
    Vector3& closestPoint)
{
    Vector3 e0 = triV1; e0.Sub(triV0);    // e0 = v1 - v0
    Vector3 e1 = triV2; e1.Sub(triV1);    // e1 = v2 - v1
    Vector3 e2 = triV0; e2.Sub(triV2);    // e2 = v0 - v2

    // tri point deltas
    Vector3 dPv0 = point; dPv0.Sub(triV0);   // dPv0 = P - v0
    Vector3 dPv1 = point; dPv1.Sub(triV1);   // dPv1 = P - v1
    Vector3 dPv2 = point; dPv2.Sub(triV2);   // dPv2 = P - v2

    // tri normal
    Vector3 n = Vector3::Cross(e0, e1); 

#ifdef CTRIGRID_TRIPOINTDISTQUERY_SSE
    // ~40% less time (on average for all code paths) in i7
    // NOTE: the SSE version is re-using XMM registers and it is quite confusing
    // look in the fpu version for better understanding the steps

    // broadcast tri normal n to 3 different registers
    const __m128 nX = _mm_set_ps1(n.x);
    const __m128 nY = _mm_set_ps1(n.y);
    const __m128 nZ = _mm_set_ps1(n.z);

    // set 0 to low float, will be masked later
    const __m128 eX = _mm_set_ps(e2.x, e1.x, e0.x, 0.f);
    const __m128 eY = _mm_set_ps(e2.y, e1.y, e0.y, 0.f);
    const __m128 eZ = _mm_set_ps(e2.z, e1.z, e0.z, 0.f);

    // compute side normals, including normalization
    // compute cross products
    const __m128 cX = _mm_sub_ps(_mm_mul_ps(eY, nZ), _mm_mul_ps(eZ, nY));
    const __m128 cY = _mm_sub_ps(_mm_mul_ps(eZ, nX), _mm_mul_ps(eX, nZ));
    const __m128 cZ = _mm_sub_ps(_mm_mul_ps(eX, nY), _mm_mul_ps(eY, nX));

    // aggregate cross product vectors with normal n in register
    // RX, RY, RZ now contain the xyz of side normals R0, R1, R2 and tri normal n
    __m128 RX = _mm_move_ss(cX, nX);
    __m128 RY = _mm_move_ss(cY, nY);
    __m128 RZ = _mm_move_ss(cZ, nZ);

    // compute reciprocal norm for all vectors
    __m128 Wnorms = _mm_mul_ps(RX, RX);
    Wnorms = _mm_add_ps(_mm_mul_ps(RY, RY), Wnorms);
    Wnorms = _mm_add_ps(_mm_mul_ps(RZ, RZ), Wnorms);
    Wnorms = _mm_rsqrt_ps(Wnorms);

    // normalize previous normals 
    RX = _mm_mul_ps(RX, Wnorms);
    RY = _mm_mul_ps(RY, Wnorms);
    RZ = _mm_mul_ps(RZ, Wnorms);

    // point projections
    // use set() instead of load() to avoid shuffles
    const __m128 lX = _mm_set_ps(dPv2.x, dPv1.x, dPv0.x, dPv0.x);
    const __m128 lY = _mm_set_ps(dPv2.y, dPv1.y, dPv0.y, dPv0.y);
    const __m128 lZ = _mm_set_ps(dPv2.z, dPv1.z, dPv0.z, dPv0.z);

    __m128 W = _mm_mul_ps(lX, RX);
    W = _mm_add_ps(_mm_mul_ps(lY, RY), W);
    W = _mm_add_ps(_mm_mul_ps(lZ, RZ), W);

    Vector4 dots;
    _mm_store_ps(dots.m_elems, W);
    const float d = dots.x;
    const float d_R0 = dots.y;
    const float d_R1 = dots.z;
    const float d_R2 = dots.w;

    // compute the angle dot products
    // re-arrange to go from wzyx -> ywz0 to do the dot products (x is not important)
    __m128 Xs = CTRIGRID_MATH_SSE_SHIFT_LEFT(_mm_movehl_ps(CTRIGRID_MATH_SSE_SHIFT_LEFT(RX, 4u), RX), 4u);
    __m128 Ys = CTRIGRID_MATH_SSE_SHIFT_LEFT(_mm_movehl_ps(CTRIGRID_MATH_SSE_SHIFT_LEFT(RY, 4u), RY), 4u);
    __m128 Zs = CTRIGRID_MATH_SSE_SHIFT_LEFT(_mm_movehl_ps(CTRIGRID_MATH_SSE_SHIFT_LEFT(RZ, 4u), RZ), 4u);

    W = _mm_mul_ps(RX, Xs);
    W = _mm_add_ps(_mm_mul_ps(RY, Ys), W);
    W = _mm_add_ps(_mm_mul_ps(RZ, Zs), W);
    _mm_store_ps(dots.m_elems, W);

    const float a01 = dots.y;
    const float a12 = dots.z;
    const float a20 = dots.w;

#else
    // side plane normals
    Vector3 n_R0 = Vector3::Cross(e0, n);
    Vector3 n_R1 = Vector3::Cross(e1, n);
    Vector3 n_R2 = Vector3::Cross(e2, n);

    n.Normalize();
    n_R0.Normalize();
    n_R1.Normalize();
    n_R2.Normalize();

    // point projections
    const float d = dPv0.Dot(n);
    const float d_R0 = dPv0.Dot(n_R0);
    const float d_R1 = dPv1.Dot(n_R1);
    const float d_R2 = dPv2.Dot(n_R2);

    // angle dot products
    const float a01 = n_R0.Dot(n_R1);
    const float a12 = n_R1.Dot(n_R2);
    const float a20 = n_R2.Dot(n_R0);
#endif

    // flags to switch the appropriate code path
    const uint32_t flags =  (d_R2 > 0.f ?       1u : 0u) |
                            (d_R1 > 0.f ? 1u << 1u : 0u) | 
                            (d_R0 > 0.f ? 1u << 2u : 0u);

    switch (flags)
    {
    // closest point is on the tri plane
    case 0u:
        {
#ifdef CTRIGRID_TRIPOINTDISTQUERY_SSE
            // multiply in fpu to avoid shuffles & stores
            Vector4 norms;
            //_mm_store_ps(norms.m_elems, Wnorms);
            _mm_store_ps1(norms.m_elems, Wnorms);
            n.Mul(norms.x);
#endif

            // C = P - d * n
            closestPoint = point;
            closestPoint.MulAdd(-d, n);
        }
        break;
    
    // only in the positive side of one side lane
    case 1u: // 001
        // check if closest to edge e2
        s_ClosestPointLineSegment(dPv2, e2, triV2, triV0, closestPoint);
        break;
    case 2u: // 010
        // check if closest to edge e1
        s_ClosestPointLineSegment(dPv1, e1, triV1, triV2, closestPoint);
        break;
    case 4u: // 100
        // check if closest to edge e0
        s_ClosestPointLineSegment(dPv0, e0, triV0, triV1, closestPoint);
        break;
    
    // in the positive side of two side planes
    // can chose a point for acute angles, need to check two edges if not
    case 6u: // 110
        {
            if (a01 < 0.f)  // acute angle
                closestPoint = triV1;
            else
            {
                if (s_ClosestPointLineSegment(dPv0, e0, triV0, triV1, closestPoint) != eCLOSEST_POINT_ON_SEGMENT)
                    s_ClosestPointLineSegment(dPv1, e1, triV1, triV2, closestPoint);
            }
        }
        break;
    case 3u: // 011
        {
            if (a12 < 0.f)  // acute angle
                closestPoint = triV2;
            else
            {
                if (s_ClosestPointLineSegment(dPv1, e1, triV1, triV2, closestPoint) != eCLOSEST_POINT_ON_SEGMENT)
                    s_ClosestPointLineSegment(dPv2, e2, triV2, triV0, closestPoint);
            }
        }
        break;
    case 5u: // 101
        {
            if (a20 < 0.f)  // acute angle
                closestPoint = triV0;
            else
            {
                if (s_ClosestPointLineSegment(dPv2, e2, triV2, triV0, closestPoint) != eCLOSEST_POINT_ON_SEGMENT)
                    s_ClosestPointLineSegment(dPv0, e0, triV0, triV1, closestPoint);
            }
        }
        break;

    default:
        break;
    }

    return true;
}

bool 
ClosestDistanceQuery::ClosestPointOnAxisAlignedBox(
    const Vector3& point, const AxisAlignedBoundingBox& bbox, Vector3& closestPoint)
{
    const Vector3 boxCenter = bbox.ComputeCenter();
    const Vector3 boxExtends = bbox.ComputeExtends();

    Vector3 p = point;
    p.Sub(boxCenter);

    // TODO: SSE
    const float xd = p.x > 0.f ? boxExtends.x : -boxExtends.x;
    const float yd = p.y > 0.f ? boxExtends.y : -boxExtends.y;
    const float zd = p.z > 0.f ? boxExtends.z : -boxExtends.z;

    p.x = fabsf(p.x) > boxExtends.x ? xd : p.x;
    p.y = fabsf(p.y) > boxExtends.y ? yd : p.y;
    p.z = fabsf(p.z) > boxExtends.z ? zd : p.z;

    closestPoint = p;
    closestPoint.Add(boxCenter);

    return true;
}

}