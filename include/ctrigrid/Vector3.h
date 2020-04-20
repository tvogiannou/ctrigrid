#pragma  once

#include <ctrigrid/Compiler.h>

#include <cmath>
#include <algorithm>


namespace ctrigrid
{

class Vector3
{
public:

#ifdef _WIN32
__pragma(warning(push))
__pragma(warning(disable : 4201))   // C4201: nameless union as non standard extension 
#endif
    union alignas(16) 
    {
        struct { float x, y, z; };
        float m_elems[4]; // last element is used for alignment padding
    };
#ifdef _WIN32
__pragma(warning(pop))
#endif

public:
    // constant values
    /// Vector (0,0,0)
    static const Vector3 ZERO;
    /// Vector (1,1,1)
    static const Vector3 UNARY;
    /// Vector (1,0,0)
    static const Vector3 UNIT_X;
    /// Vector (0,1,0)
    static const Vector3 UNIT_Y;
    /// Vector (0,0,1)
    static const Vector3 UNIT_Z;
    /// Vector (-1,0,0)
    static const Vector3 NEG_UNIT_X;
    /// Vector (0,-1,0)
    static const Vector3 NEG_UNIT_Y;
    /// Vector (0,0,-1)
    static const Vector3 NEG_UNIT_Z;

public:
    // construction

    /// Default constructor
    Vector3() = default;
    /// Constructor from 3 values
    Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    explicit Vector3(float a) : x(a), y(a), z(a) {}

public:
    // math operations
    void Sub(const Vector3& s) { x -= s.x; y -= s.y; z -= s.z; }
    void Add(const Vector3& s) { x += s.x; y += s.y; z += s.z; }
    void Mul(const Vector3& s) { x *= s.x; y *= s.y; z *= s.z; }
    void Div(const Vector3& s) { x /= s.x; y /= s.y; z /= s.z; }
    void Sub(float a) { x -= a; y -= a; z -= a; }
    void Add(float a) { x += a; y += a; z += a; }
    void Mul(float a) { x *= a; y *= a; z *= a; }
    void Div(float a) { x /= a; y /= a; z /= a; }
    void MulAdd(float a, const Vector3 &s) { x += a * s.x; y += a * s.y; z += a * s.z; }

    // special math operations
    void Abs() { x = fabsf(x); y = fabsf(y); z = fabsf(z); }
    static Vector3 Min(const Vector3& s1, const Vector3& s2) { return Vector3(std::min<float>(s1.x, s2.x), std::min<float>(s1.y, s2.y), std::min<float>(s1.z, s2.z)); }
    static Vector3 Max(const Vector3& s1, const Vector3& s2) { return Vector3(std::max<float>(s1.x, s2.x), std::max<float>(s1.y, s2.y), std::max<float>(s1.z, s2.z)); }

    /// invert components in place
    void Invert() { x = -x; y = -y; z = -z; }
    void InvertX() { x = -x; }
    void InvertY() { y = -y; }
    void InvertZ() { z = -z; }

    // Dot products
    float Dot() const { return x * x + y * y + z * z; }
    float Dot(const Vector3& other) const { return x * other.x + y * other.y + z * other.z; }

    // Coordinates sum
    float Sum() const { return x + y + z; }
    // Vector normalization
    void Normalize() { Div(Norm()); }
    // Vector norm
    float Norm() const { return sqrtf(Dot()); }
    // The normal in the direction of vector
    Vector3 Normal() const 
    { 
        Vector3 temp(*this);
        temp.Normalize(); 
        return temp; 
    }
    // Distance with the other vector
    float Dist(const Vector3 &other) const
    { 
        Vector3 temp(*this);
        temp.Sub(other);
        return temp.Norm();
    }

    // Cross product
    static Vector3 Cross(const Vector3 &s1, const Vector3 &s2)
    {
        return Vector3(s1.y*s2.z - s1.z*s2.y, s1.z*s2.x - s1.x*s2.z, s1.x*s2.y - s1.y*s2.x);
    }
};

}
