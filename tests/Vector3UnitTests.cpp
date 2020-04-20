
#include <gtest/gtest.h>

#include <ctrigrid/Vector3.h>

#include "MathTestUtils.h"



TEST(Vector3UnitTests, TestVector3Constructors)
{
	using namespace ctrigrid;

	{
		Vector3 v(1.f, 2.f, 3.f);
		mathtests::Utils::CheckValues(v, 1.f, 2.f, 3.f);
	}

	{
		const float value = 3.14f;
		Vector3 v(value);
		mathtests::Utils::CheckValues(v, value, value, value);
	}
}

TEST(Vector3UnitTests, TestVector3Constants)
{
	using namespace ctrigrid;

	{
		Vector3 v = Vector3::ZERO;
		mathtests::Utils::CheckValues(v, 0.f, 0.f, 0.f);
	}

	{
		Vector3 v = Vector3::UNARY;
		mathtests::Utils::CheckValues(v, 1.f, 1.f, 1.f);
	}

	{
		Vector3 v = Vector3::UNIT_X;
		mathtests::Utils::CheckValues(v, 1.f, 0.f, 0.f);
	}

	{
		Vector3 v = Vector3::UNIT_Y;
		mathtests::Utils::CheckValues(v, 0.f, 1.f, 0.f);
	}

	{
		Vector3 v = Vector3::UNIT_Z;
		mathtests::Utils::CheckValues(v, 0.f, 0.f, 1.f);
	}

	{
		Vector3 v = Vector3::NEG_UNIT_X;
		mathtests::Utils::CheckValues(v, -1.f, 0.f, 0.f);
	}

	{
		Vector3 v = Vector3::NEG_UNIT_Y;
		mathtests::Utils::CheckValues(v, 0.f, -1.f, 0.f);
	}

	{
		Vector3 v = Vector3::NEG_UNIT_Z;
		mathtests::Utils::CheckValues(v, 0.f, 0.f, -1.f);
	}
}

TEST(Vector3UnitTests, TestVector3Vector3ArithmeticOperations)
{
	using namespace ctrigrid;

	const Vector3 v1(1.f, 2.f, 3.f);
	const Vector3 v2(3.f, 2.f, 1.f);

	{
		Vector3 v = v1;
		v.Sub(v2);
		mathtests::Utils::CheckValues(v, -2.f, 0.f, 2.f);
	}

	{
		Vector3 v = v1;
		v.Add(v2);
		mathtests::Utils::CheckValues(v, 4.f, 4.f, 4.f);
	}

	{
		Vector3 v = v1;
		v.Mul(v2);
		mathtests::Utils::CheckValues(v, 3.f, 4.f, 3.f);
	}

	{
		Vector3 v = v1;
		v.Div(v2);
		mathtests::Utils::CheckValues(v, 1.f/3.f, 1.f, 3.f);
	}

    {
        const float f = 2.f;
        Vector3 v = v1;
        v.MulAdd(f, v2);
        mathtests::Utils::CheckValues(v, 7.f, 6.f, 5.f);
    }

    {
        Vector3 v(-7.f, -6.f, 5.f);
        v.Abs();
        mathtests::Utils::CheckValues(v, 7.f, 6.f, 5.f);
    }

    {
        Vector3 v = Vector3::Min(v1, v2);
        mathtests::Utils::CheckValues(v, 1.f, 2.f, 1.f);
    }

    {
        Vector3 v = Vector3::Max(v1, v2);
        mathtests::Utils::CheckValues(v, 3.f, 2.f, 3.f);
    }
}

TEST(Vector3UnitTests, TestVector3ScalarArithmeticOperations)
{
	using namespace ctrigrid;

	const Vector3 v1(1.f, 2.f, 3.f);
	const float value = 2.f;

	{
		Vector3 v = v1;
		v.Sub(value);
		mathtests::Utils::CheckValues(v, -1.f, 0.f, 1.f);
	}

	{
		Vector3 v = v1;
		v.Add(value);
		mathtests::Utils::CheckValues(v, 3.f, 4.f, 5.f);
	}

	{
		Vector3 v = v1;
		v.Mul(value);
		mathtests::Utils::CheckValues(v, 2.f, 4.f, 6.f);
	}

	{
		Vector3 v = v1;
		v.Div(value);
		mathtests::Utils::CheckValues(v, 0.5f, 1.f, 1.5f);
	}
}

TEST(Vector3UnitTests, TestVector3Inversion)
{
	using namespace ctrigrid;
	
	const Vector3 v1(1.f, 2.f, 3.f);

	{
		Vector3 v = v1;
		v.Invert();
		mathtests::Utils::CheckValues(v, -1.f, -2.f, -3.f);
	}

	{
		Vector3 v = v1;

		v.InvertX();
		mathtests::Utils::CheckValues(v, -1.f, 2.f, 3.f);

		v.InvertY();
		mathtests::Utils::CheckValues(v, -1.f, -2.f, 3.f);

		v.InvertX();
		v.InvertZ();
		mathtests::Utils::CheckValues(v, 1.f, -2.f, -3.f);
	}
}

TEST(Vector3UnitTests, TestVector3Products)
{
	using namespace ctrigrid;

	const Vector3 v1(1.f, 2.f, 3.f);
	const Vector3 v2(3.f, 2.f, 1.f);

	{
		const float sum1 = v1.Sum();
		const float sum2 = v2.Sum();

		EXPECT_FLOAT_EQ(sum1, 6.f);
		EXPECT_FLOAT_EQ(sum1, sum2);
	}

	{
		const float dot1 = v1.Dot();
		const float dot2 = v2.Dot();
		const float dot11 = v1.Dot(v1);
		const float dot22 = v2.Dot(v2);

		EXPECT_FLOAT_EQ(dot1, 14.f);
		EXPECT_FLOAT_EQ(dot1, dot2);
		EXPECT_FLOAT_EQ(dot11, dot1);
		EXPECT_FLOAT_EQ(dot22, dot2);
	}

	{
		const float dot12 = v1.Dot(v2);
		const float dot21 = v2.Dot(v1);

		EXPECT_FLOAT_EQ(dot12, 10.f);
		EXPECT_FLOAT_EQ(dot12, dot21);
	}

	{
		const Vector3 expCross(-4.f, 8.f, -4.f);
		const Vector3 c = Vector3::Cross(v1, v2);
		mathtests::Utils::CheckValues(c, expCross);
	}

	{
		// cross units relation
		const Vector3 xy = Vector3::Cross(Vector3::UNIT_X, Vector3::UNIT_Y);
		const Vector3 yz = Vector3::Cross(Vector3::UNIT_Y, Vector3::UNIT_Z);
		const Vector3 zx = Vector3::Cross(Vector3::UNIT_Z, Vector3::UNIT_X);

		mathtests::Utils::CheckValues(xy, Vector3::UNIT_Z);
		mathtests::Utils::CheckValues(yz, Vector3::UNIT_X);
		mathtests::Utils::CheckValues(zx, Vector3::UNIT_Y);
	}
}

TEST(Vector3UnitTests, TestVector3Normalize)
{
	using namespace ctrigrid;

	const Vector3 v1(1.f, 2.f, 3.f);
	const Vector3 v2(3.f, 2.f, 1.f);
	const float expV = sqrtf(14.f);

	{
		const float norm1 = v1.Norm();
		const float norm2 = v2.Norm();

		EXPECT_FLOAT_EQ(norm1, expV);
		EXPECT_FLOAT_EQ(norm1, norm2);
	}

	{
		Vector3 n = v1;
		n.Normalize();
		mathtests::Utils::CheckValues(n, 1.f/expV, 2.f/expV, 3.f/expV);

		const Vector3 n1 = v1.Normal();
		mathtests::Utils::CheckValues(n, n1);
	}

	{
		const float expD = sqrtf(8.f);
		const float d12 = v1.Dist(v2);
		const float d21 = v2.Dist(v1);

		EXPECT_FLOAT_EQ(d12, expD);
		EXPECT_FLOAT_EQ(d12, d21);
	}
}