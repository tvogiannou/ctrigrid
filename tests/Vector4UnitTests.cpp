
#include <gtest/gtest.h>

#include <ctrigrid/Vector4.h>

#include "MathTestUtils.h"



TEST(Vector4UnitTests, TestVector4Constructors)
{
	using namespace ctrigrid;

	{
		Vector4 v(1.f, 2.f, 3.f, 4.f);
		mathtests::Utils::CheckValues(v, 1.f, 2.f, 3.f, 4.f);
	}

	{
		const float value = 3.14f;
		Vector4 v(value);
		mathtests::Utils::CheckValues(v, value, value, value, value);
	}

	{
		const Vector3 vec3(1.f, 2.f, 3.f);
		Vector4 v(vec3, 4.f);
		mathtests::Utils::CheckValues(v, vec3.x, vec3.y, vec3.z, 4.f);
	}
}

TEST(Vector4UnitTests, TestVector4Constants)
{
	using namespace ctrigrid;

	{
		Vector4 v = Vector4::ZERO;
		mathtests::Utils::CheckValues(v, 0.f, 0.f, 0.f, 0.f);
	}

	{
		Vector4 v = Vector4::UNARY;
		mathtests::Utils::CheckValues(v, 1.f, 1.f, 1.f, 1.f);
	}

	{
		Vector4 v = Vector4::UNIT_X;
		mathtests::Utils::CheckValues(v, 1.f, 0.f, 0.f, 0.f);
	}

	{
		Vector4 v = Vector4::UNIT_Y;
		mathtests::Utils::CheckValues(v, 0.f, 1.f, 0.f, 0.f);
	}

	{
		Vector4 v = Vector4::UNIT_Z;
		mathtests::Utils::CheckValues(v, 0.f, 0.f, 1.f, 0.f);
	}

	{
		Vector4 v = Vector4::UNIT_W;
		mathtests::Utils::CheckValues(v, 0.f, 0.f, 0.f, 1.f);
	}

	{
		Vector4 v = Vector4::NEG_UNIT_X;
		mathtests::Utils::CheckValues(v, -1.f, 0.f, 0.f, 0.f);
	}

	{
		Vector4 v = Vector4::NEG_UNIT_Y;
		mathtests::Utils::CheckValues(v, 0.f, -1.f, 0.f, 0.f);
	}

	{
		Vector4 v = Vector4::NEG_UNIT_Z;
		mathtests::Utils::CheckValues(v, 0.f, 0.f, -1.f, 0.f);
	}

	{
		Vector4 v = Vector4::NEG_UNIT_W;
		mathtests::Utils::CheckValues(v, 0.f, 0.f, 0.f, -1.f);
	}
}

TEST(Vector4UnitTests, TestVector4Vector4ArithmeticOperations)
{
	using namespace ctrigrid;

	const Vector4 v1(1.f, 2.f, 3.f, 4.f);
	const Vector4 v2(3.f, 2.f, 1.f, -1.f);

	{
		Vector4 v = v1;
		v.Sub(v2);
		mathtests::Utils::CheckValues(v, -2.f, 0.f, 2.f, 5.f);
	}

	{
		Vector4 v = v1;
		v.Add(v2);
		mathtests::Utils::CheckValues(v, 4.f, 4.f, 4.f, 3.f);
	}

	{
		Vector4 v = v1;
		v.Mul(v2);
		mathtests::Utils::CheckValues(v, 3.f, 4.f, 3.f, -4.f);
	}

	{
		Vector4 v = v1;
		v.Div(v2);
		mathtests::Utils::CheckValues(v, 1.f/3.f, 1.f, 3.f, -4.f);
	}
}

TEST(Vector4UnitTests, TestVector4ScalarArithmeticOperations)
{
	using namespace ctrigrid;

	const Vector4 v1(1.f, 2.f, 3.f, 4.f);
	const float value = 2.f;

	{
		Vector4 v = v1;
		v.Sub(value);
		mathtests::Utils::CheckValues(v, -1.f, 0.f, 1.f, 2.f);
	}

	{
		Vector4 v = v1;
		v.Add(value);
		mathtests::Utils::CheckValues(v, 3.f, 4.f, 5.f, 6.f);
	}

	{
		Vector4 v = v1;
		v.Mul(value);
		mathtests::Utils::CheckValues(v, 2.f, 4.f, 6.f, 8.f);
	}

	{
		Vector4 v = v1;
		v.Div(value);
		mathtests::Utils::CheckValues(v, 0.5f, 1.f, 1.5f, 2.f);
	}
}

TEST(Vector4UnitTests, TestVector4Inversion)
{
	using namespace ctrigrid;
	
	const Vector4 v1(1.f, 2.f, 3.f, 4.f);

	{
		Vector4 v = v1;
		v.Invert();
		mathtests::Utils::CheckValues(v, -1.f, -2.f, -3.f, -4.f);
	}

	{
		Vector4 v = v1;

		v.InvertX();
		mathtests::Utils::CheckValues(v, -1.f, 2.f, 3.f, 4.f);

		v.InvertY();
		mathtests::Utils::CheckValues(v, -1.f, -2.f, 3.f, 4.f);

		v.InvertX();
		v.InvertZ();
		mathtests::Utils::CheckValues(v, 1.f, -2.f, -3.f, 4.f);

		v.InvertY();
		v.InvertW();
		mathtests::Utils::CheckValues(v, 1.f, 2.f, -3.f, -4.f);
	}
}

TEST(Vector4UnitTests, TestVector4Products)
{
	using namespace ctrigrid;

	const Vector4 v1(1.f, 2.f, 3.f, 4.f);
	const Vector4 v2(3.f, 2.f, 1.f, 4.f);

	{
		const float sum1 = v1.Sum();
		const float sum2 = v2.Sum();

		EXPECT_FLOAT_EQ(sum1, 10.f);
		EXPECT_FLOAT_EQ(sum1, sum2);
	}

	{
		const float dot1 = v1.Dot();
		const float dot2 = v2.Dot();
		const float dot11 = v1.Dot(v1);
		const float dot22 = v2.Dot(v2);

		EXPECT_FLOAT_EQ(dot1, 30.f);
		EXPECT_FLOAT_EQ(dot1, dot2);
		EXPECT_FLOAT_EQ(dot11, dot1);
		EXPECT_FLOAT_EQ(dot22, dot2);
	}

	{
		const float dot12 = v1.Dot(v2);
		const float dot21 = v2.Dot(v1);

		EXPECT_FLOAT_EQ(dot12, 26.f);
		EXPECT_FLOAT_EQ(dot12, dot21);
	}
}

TEST(Vector4UnitTests, TestVector4Normalize)
{
	using namespace ctrigrid;

	const Vector4 v1(1.f, 2.f, 3.f, 4.f);
	const Vector4 v2(3.f, 2.f, 1.f, 4.f);
	const float expV = sqrtf(30.f);

	{
		const float norm1 = v1.Norm();
		const float norm2 = v2.Norm();

		EXPECT_FLOAT_EQ(norm1, expV);
		EXPECT_FLOAT_EQ(norm1, norm2);
	}

	{
		Vector4 n = v1;
		n.Normalize();
		mathtests::Utils::CheckValues(n, 1.f/expV, 2.f/expV, 3.f/expV, 4.f/expV);

		const Vector4 n1 = v1.Normal();
		mathtests::Utils::CheckValues(n, n1);
	}
}
