
#pragma once

#include <ctrigrid/Vector3.h>
#include <ctrigrid/Vector4.h>

#include <gtest/gtest.h>


namespace mathtests
{

struct Utils
{
    static void CheckValues(const ctrigrid::Vector3& v, float expX, float expY, float expZ)
    {
        EXPECT_FLOAT_EQ(v.x, expX);
        EXPECT_FLOAT_EQ(v.y, expY);
        EXPECT_FLOAT_EQ(v.z, expZ);
    
        EXPECT_FLOAT_EQ(v.m_elems[0], expX);
        EXPECT_FLOAT_EQ(v.m_elems[1], expY);
        EXPECT_FLOAT_EQ(v.m_elems[2], expZ);
    }
    
    static void CheckValues(const ctrigrid::Vector3& v1, const ctrigrid::Vector3& v2)
    {
        EXPECT_FLOAT_EQ(v1.x, v2.x);
        EXPECT_FLOAT_EQ(v1.y, v2.y);
        EXPECT_FLOAT_EQ(v1.z, v2.z);
    
        EXPECT_FLOAT_EQ(v1.m_elems[0], v2.m_elems[0]);
        EXPECT_FLOAT_EQ(v1.m_elems[1], v2.m_elems[1]);
        EXPECT_FLOAT_EQ(v1.m_elems[2], v2.m_elems[2]);
    }

    static void CheckValues(const ctrigrid::Vector3& v1, const ctrigrid::Vector3& v2, float tolerance)
    {
        EXPECT_NEAR(v1.x, v2.x, tolerance);
        EXPECT_NEAR(v1.y, v2.y, tolerance);
        EXPECT_NEAR(v1.z, v2.z, tolerance);

        EXPECT_NEAR(v1.m_elems[0], v2.m_elems[0], tolerance);
        EXPECT_NEAR(v1.m_elems[1], v2.m_elems[1], tolerance);
        EXPECT_NEAR(v1.m_elems[2], v2.m_elems[2], tolerance);
    }

    static void CheckValues(const ctrigrid::Vector4& v, float expX, float expY, float expZ, float expW)
    {
        EXPECT_FLOAT_EQ(v.x, expX);
        EXPECT_FLOAT_EQ(v.y, expY);
        EXPECT_FLOAT_EQ(v.z, expZ);
        EXPECT_FLOAT_EQ(v.w, expW);

        EXPECT_FLOAT_EQ(v.m_elems[0], expX);
        EXPECT_FLOAT_EQ(v.m_elems[1], expY);
        EXPECT_FLOAT_EQ(v.m_elems[2], expZ);
        EXPECT_FLOAT_EQ(v.m_elems[3], expW);
    }

    static void CheckValues(const ctrigrid::Vector4& v1, const ctrigrid::Vector4& v2)
    {
        EXPECT_FLOAT_EQ(v1.x, v2.x);
        EXPECT_FLOAT_EQ(v1.y, v2.y);
        EXPECT_FLOAT_EQ(v1.z, v2.z);
        EXPECT_FLOAT_EQ(v1.w, v2.w);

        EXPECT_FLOAT_EQ(v1.m_elems[0], v2.m_elems[0]);
        EXPECT_FLOAT_EQ(v1.m_elems[1], v2.m_elems[1]);
        EXPECT_FLOAT_EQ(v1.m_elems[2], v2.m_elems[2]);
        EXPECT_FLOAT_EQ(v1.m_elems[3], v2.m_elems[3]);
    }
};

}