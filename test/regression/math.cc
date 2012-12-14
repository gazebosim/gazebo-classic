/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <string.h>
#include "common/Console.hh"
#include "math/gzmath.hh"
#include "msgs/msgs.hh"
#include "ServerFixture.hh"

using namespace gazebo;
class MathTest : public ServerFixture
{
};

TEST_F(MathTest, Box)
{
  {
    math::Box box;
    EXPECT_TRUE(box.min == math::Vector3(0, 0, 0));
    EXPECT_TRUE(box.max == math::Vector3(0, 0, 0));
  }

  {
    math::Box box(math::Vector3(0, 1, 2), math::Vector3(1, 2, 3));
    EXPECT_TRUE(box.min == math::Vector3(0, 1, 2));
    EXPECT_TRUE(box.max == math::Vector3(1, 2, 3));
  }

  {
    math::Box box(math::Vector3(0, 1, 2), math::Vector3(1, 2, 3));
    math::Box box1(box);
    EXPECT_TRUE(box1.min == box.min);
    EXPECT_TRUE(box1.max == box.max);
  }

  math::Box box(math::Vector3(0, 1, 2), math::Vector3(1, 2, 3));
  EXPECT_EQ(box.GetXLength(), 1);
  EXPECT_EQ(box.GetYLength(), 1);
  EXPECT_EQ(box.GetZLength(), 1);
  EXPECT_TRUE(box.GetSize() == math::Vector3(1, 1, 1));
  EXPECT_TRUE(box.GetCenter() == math::Vector3(0.5, 1.5, 2.5));

  box.Merge(math::Box(math::Vector3(-1, -1, -1), math::Vector3(2, 2, 2)));
  EXPECT_TRUE(box == math::Box(math::Vector3(-1, -1, -1),
                               math::Vector3(2, 2, 3)));

  box = math::Box(math::Vector3(1, 1, 1), math::Vector3(3, 3, 3));
  EXPECT_TRUE(box == math::Box(math::Vector3(1, 1, 1), math::Vector3(3, 3, 3)));

  box += math::Box(math::Vector3(2, 2, 2), math::Vector3(4, 4, 4));
  EXPECT_TRUE(box == math::Box(math::Vector3(1, 1, 1), math::Vector3(4, 4, 4)));

  box = box + math::Box(math::Vector3(-2, -2, -2), math::Vector3(4, 4, 4));
  EXPECT_TRUE(box == math::Box(math::Vector3(-2, -2, -2),
                               math::Vector3(4, 4, 4)));
}

TEST_F(MathTest, Vector4)
{
  {
    math::Vector4 v;
    EXPECT_TRUE(math::equal(v.x, 0.0));
    EXPECT_TRUE(math::equal(v.y, 0.0));
    EXPECT_TRUE(math::equal(v.z, 0.0));
    EXPECT_TRUE(math::equal(v.w, 0.0));
  }

  math::Vector4 v1(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(v1.x, 1.0));
  EXPECT_TRUE(math::equal(v1.y, 2.0));
  EXPECT_TRUE(math::equal(v1.z, 3.0));
  EXPECT_TRUE(math::equal(v1.w, 4.0));

  math::Vector4 v(v1);
  EXPECT_TRUE(v == v1);

  EXPECT_TRUE(math::equal(v.Distance(math::Vector4(0, 0, 0, 0)), 5.4772, 1e-3));

  // ::GetLength()
  v.Set(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(v.GetLength(), 5.4772, 1e-3));

  // ::GetSquaredLength()
  EXPECT_TRUE(math::equal(v.GetSquaredLength(), 30.0));

  // ::Normalize
  v.Normalize();
  EXPECT_TRUE(v == math::Vector4(0.182574, 0.365148, 0.547723, 0.730297));

  // ::Set
  v.Set(2, 4, 6, 8);
  EXPECT_TRUE(v == math::Vector4(2, 4, 6, 8));

  // ::operator= vector4
  v = v1;
  EXPECT_TRUE(v == v1);

  // ::operator= double
  v = 1.2;
  EXPECT_TRUE(v == math::Vector4(1.2, 1.2, 1.2, 1.2));

  // ::operator+ vector4
  v = v + math::Vector4(1, 2, 3, 4);
  EXPECT_TRUE(v == math::Vector4(2.2, 3.2, 4.2, 5.2));

  // ::operator+=
  v += v;
  EXPECT_TRUE(v == math::Vector4(4.4, 6.4, 8.4, 10.4));

  // ::operator- vector4
  v = v - math::Vector4(1, 1, 1, 1);
  EXPECT_TRUE(v == math::Vector4(3.4, 5.4, 7.4, 9.4));

  // ::operator-= vector4
  v -= math::Vector4(1, 1, 1, 1);
  EXPECT_TRUE(v == math::Vector4(2.4, 4.4, 6.4, 8.4));


  // ::operator/ vector4
  v = v / math::Vector4(.1, .1, .1, .1);
  EXPECT_TRUE(v == math::Vector4(24, 44, 64, 84));

  // ::operator/ double
  v = v / 2.0;
  EXPECT_TRUE(v == math::Vector4(12, 22, 32, 42));

  // ::operator/= vector4
  v /= math::Vector4(4, 4, 4, 4);
  EXPECT_TRUE(v == math::Vector4(3, 5.5, 8, 10.5));

  // ::operator/= double
  v /= .1;
  EXPECT_TRUE(v == math::Vector4(30, 55, 80, 105));

  // ::operator * matrix4
  v = v * math::Matrix4(1, 2, 3, 4,
                        5, 6, 7, 8,
                        9, 10, 11, 12,
                        13, 14, 15, 16);
  EXPECT_TRUE(v == math::Vector4(2390, 2660, 2930, 3200));


  // ::operator * vector4
  v = v * math::Vector4(.2, .3, .4, .5);
  EXPECT_TRUE(v == math::Vector4(478, 798, 1172, 1600));

  // ::operator *= vector4
  v *= math::Vector4(2, 4, 6, 8);
  EXPECT_TRUE(v == math::Vector4(956, 3192, 7032, 12800));

  // ::operator * double
  v = v * 5.2;
  EXPECT_TRUE(v == math::Vector4(4971.2, 16598.4, 36566.4, 66560));

  // ::operator *= double
  v *= 10.0;
  EXPECT_TRUE(v == math::Vector4(49712, 1.65984e+05, 3.65664e+05, 6.656e+05));

  // ::operator != vector4
  EXPECT_TRUE(v != math::Vector4());

  // ::IsFinite
  EXPECT_TRUE(v.IsFinite());

  // ::operator[]
  v.Set(1, 2, 3, 4);
  EXPECT_EQ(v[0], 1);
  EXPECT_EQ(v[1], 2);
  EXPECT_EQ(v[2], 3);
  EXPECT_EQ(v[3], 4);
}

/////////////////////////////////////////////////
TEST_F(MathTest, Vector3)
{
  math::Vector3 v;

  // ::Distance, ::GetLEngth()
  v.Set(1, 2, 3);
  EXPECT_TRUE(v.GetLength() == v.Distance(math::Vector3(0, 0, 0)));

  // ::GetRound
  v.Set(1.23, 2.34, 3.55);
  EXPECT_TRUE(v.GetRounded() == math::Vector3(1, 2, 4));

  // ::Round
  v.Round();
  EXPECT_TRUE(v.Round() == math::Vector3(1, 2, 4));

  // ::GetDotProd
  EXPECT_TRUE(math::equal(17.0, v.Dot(math::Vector3(1, 2, 3)), 1e-2));

  // ::GetDistToLine
  v.Set(0, 0, 0);
  EXPECT_EQ(1.0, v.GetDistToLine(math::Vector3(1, -1, 0),
                                 math::Vector3(1, 1, 0)));

  // ::operator= double
  v = 4.0;
  EXPECT_TRUE(v == math::Vector3(4, 4, 4));

  // ::operator/ vector3
  v = v / math::Vector3(1, 2, 4);
  EXPECT_TRUE(v == math::Vector3(4, 2, 1));

  // ::operator / double
  v = v / 2;
  EXPECT_TRUE(v == math::Vector3(2, 1, .5));

  // ::operator * vector3
  v = v * math::Vector3(2, 3, 4);
  EXPECT_TRUE(v == math::Vector3(4, 3, 2));

  // ::operator[]
  EXPECT_EQ(v[0], 4);
  EXPECT_EQ(v[1], 3);
  EXPECT_EQ(v[2], 2);

  v.Set(1.23, 2.35, 3.654321);
  v.Round(1);
  EXPECT_TRUE(v == math::Vector3(1.2, 2.4, 3.7));

  // operator GetAbs
  v.Set(-1, -2, -3);
  EXPECT_TRUE(v.GetAbs() == math::Vector3(1, 2, 3));

  // operator /=
  v.Set(1, 2, 4);
  v /= math::Vector3(1, 4, 4);
  EXPECT_TRUE(v == math::Vector3(1, .5, 1));

  // operator *=
  v.Set(1, 2, 4);
  v *= math::Vector3(2, .5, .1);
  EXPECT_TRUE(v.Equal(math::Vector3(2, 1, .4)));
}

/////////////////////////////////////////////////
TEST_F(MathTest, Vector2i)
{
  {
    math::Vector2i v;
    EXPECT_EQ(0, v.x);
    EXPECT_EQ(0, v.y);
  }

  // Constructor
  math::Vector2i v(1, 2);
  EXPECT_EQ(1, v.x);
  EXPECT_EQ(2, v.y);

  // ::Distance
  EXPECT_EQ(2, v.Distance(math::Vector2i(0, 0)));

  // ::Normalize
  v.Normalize();
  EXPECT_TRUE(v == math::Vector2i(0, 1));

  // ::Set
  v.Set(4, 5);
  EXPECT_TRUE(v == math::Vector2i(4, 5));

  // ::operator=
  v = math::Vector2i(6, 7);
  EXPECT_TRUE(v == math::Vector2i(6, 7));

  // ::operator= int
  v = 5;
  EXPECT_TRUE(v == math::Vector2i(5, 5));

  // ::operator+
  v = v + math::Vector2i(1, 2);
  EXPECT_TRUE(v == math::Vector2i(6, 7));

  // ::operator +=
  v += math::Vector2i(5, 6);
  EXPECT_TRUE(v == math::Vector2i(11, 13));

  // ::operator -
  v = v - math::Vector2i(2, 4);
  EXPECT_TRUE(v == math::Vector2i(9, 9));

  // ::operator -=
  v.Set(2, 4);
  v -= math::Vector2i(1, 6);
  EXPECT_TRUE(v == math::Vector2i(1, -2));

  // ::operator /
  v.Set(10, 6);
  v = v / math::Vector2i(2, 3);
  EXPECT_TRUE(v == math::Vector2i(5, 2));

  // ::operator /=
  v.Set(10, 6);
  v /= math::Vector2i(2, 3);
  EXPECT_TRUE(v == math::Vector2i(5, 2));

  // ::operator / int
  v.Set(10, 6);
  v = v / 2;
  EXPECT_TRUE(v == math::Vector2i(5, 3));

  // ::operator /= int
  v.Set(10, 6);
  v /= 2;
  EXPECT_TRUE(v == math::Vector2i(5, 3));

  // ::operator * int
  v.Set(10, 6);
  v = v * 2;
  EXPECT_TRUE(v == math::Vector2i(20, 12));

  // ::operator * int
  v.Set(10, 6);
  v *= 2;
  EXPECT_TRUE(v == math::Vector2i(20, 12));

  // ::operator * vector2i
  v.Set(10, 6);
  v = v * math::Vector2i(2, 4);
  EXPECT_TRUE(v == math::Vector2i(20, 24));

  // ::operator *= vector2i
  v.Set(10, 6);
  v *= math::Vector2i(2, 4);
  EXPECT_TRUE(v == math::Vector2i(20, 24));


  // ::IsFinite
  EXPECT_TRUE(v.IsFinite());

  // ::operator[]
  v.Set(6, 7);
  EXPECT_EQ(6, v[0]);
  EXPECT_EQ(7, v[1]);
}

/////////////////////////////////////////////////
TEST_F(MathTest, Vector2d)
{
  {
    math::Vector2d v;
    EXPECT_EQ(0, v.x);
    EXPECT_EQ(0, v.y);
  }

  // Constructor
  math::Vector2d v(1, 2);
  EXPECT_EQ(1, v.x);
  EXPECT_EQ(2, v.y);

  // ::Distance
  EXPECT_TRUE(math::equal(2.236, v.Distance(math::Vector2d(0, 0)), 1e-2));

  // ::Normalize
  v.Normalize();
  EXPECT_TRUE(v == math::Vector2d(0.447214, 0.894427));

  // ::Set
  v.Set(4, 5);
  EXPECT_TRUE(v == math::Vector2d(4, 5));

  // ::operator=
  v = math::Vector2d(6, 7);
  EXPECT_TRUE(v == math::Vector2d(6, 7));

  // ::operator= int
  v = 5;
  EXPECT_TRUE(v == math::Vector2d(5, 5));

  // ::operator+
  v = v + math::Vector2d(1, 2);
  EXPECT_TRUE(v == math::Vector2d(6, 7));

  // ::operator +=
  v += math::Vector2d(5, 6);
  EXPECT_TRUE(v == math::Vector2d(11, 13));

  // ::operator -
  v = v - math::Vector2d(2, 4);
  EXPECT_TRUE(v == math::Vector2d(9, 9));

  // ::operator -=
  v.Set(2, 4);
  v -= math::Vector2d(1, 6);
  EXPECT_TRUE(v == math::Vector2d(1, -2));

  // ::operator /
  v.Set(10, 6);
  v = v / math::Vector2d(2, 3);
  EXPECT_TRUE(v == math::Vector2d(5, 2));

  // ::operator /=
  v.Set(10, 6);
  v /= math::Vector2d(2, 3);
  EXPECT_TRUE(v == math::Vector2d(5, 2));

  // ::operator / int
  v.Set(10, 6);
  v = v / 2;
  EXPECT_TRUE(v == math::Vector2d(5, 3));

  // ::operator /= int
  v.Set(10, 6);
  v /= 2;
  EXPECT_TRUE(v == math::Vector2d(5, 3));

  // ::operator * int
  v.Set(10, 6);
  v = v * 2;
  EXPECT_TRUE(v == math::Vector2d(20, 12));

  // ::operator *= int
  v.Set(10, 6);
  v *= 2;
  EXPECT_TRUE(v == math::Vector2d(20, 12));

  // ::operator * vector2i
  v.Set(10, 6);
  v = v * math::Vector2d(2, 4);
  EXPECT_TRUE(v == math::Vector2d(20, 24));

  // ::operator *= vector2i
  v.Set(10, 6);
  v *= math::Vector2d(2, 4);
  EXPECT_TRUE(v == math::Vector2d(20, 24));


  // ::IsFinite
  EXPECT_TRUE(v.IsFinite());

  // ::operator[]
  v.Set(6, 7);
  EXPECT_EQ(6, v[0]);
  EXPECT_EQ(7, v[1]);
}

/////////////////////////////////////////////////
TEST_F(MathTest, Spline)
{
  math::Spline s;

  s.AddPoint(math::Vector3(0, 0, 0));
  EXPECT_EQ(static_cast<unsigned int>(1), s.GetPointCount());

  s.Clear();
  EXPECT_EQ(static_cast<unsigned int>(0), s.GetPointCount());

  s.AddPoint(math::Vector3(0, 0, 0));
  EXPECT_TRUE(s.GetPoint(0) == math::Vector3(0, 0, 0));
  s.AddPoint(math::Vector3(1, 1, 1));
  EXPECT_TRUE(s.GetPoint(1) == math::Vector3(1, 1, 1));

  // ::UpdatePoint
  s.UpdatePoint(1, math::Vector3(2, 2, 2));
  s.SetAutoCalculate(false);
  s.UpdatePoint(0, math::Vector3(-1, -1, -1));
  s.SetAutoCalculate(true);

  // ::Interpolate
  EXPECT_TRUE(s.Interpolate(0.5) == math::Vector3(0.5, 0.5, 0.5));

  // ::Interpolate
  s.AddPoint(math::Vector3(4, 4, 4));
  EXPECT_TRUE(s.Interpolate(1, 0.2) == math::Vector3(2.496, 2.496, 2.496));
}

/////////////////////////////////////////////////
TEST_F(MathTest, RotationSpline)
{
  math::RotationSpline s;

  s.AddPoint(math::Quaternion(0, 0, 0));
  EXPECT_EQ(static_cast<unsigned int>(1), s.GetNumPoints());

  s.Clear();
  EXPECT_EQ(static_cast<unsigned int>(0), s.GetNumPoints());

  s.AddPoint(math::Quaternion(0, 0, 0));
  EXPECT_TRUE(s.GetPoint(0) == math::Quaternion(0, 0, 0));
  s.AddPoint(math::Quaternion(.1, .1, .1));
  EXPECT_TRUE(s.GetPoint(1) == math::Quaternion(.1, .1, .1));

  // ::UpdatePoint
  s.UpdatePoint(1, math::Quaternion(.2, .2, .2));
  s.SetAutoCalculate(false);
  s.UpdatePoint(0, math::Vector3(-.1, -.1, -.1));
  s.SetAutoCalculate(true);

  // ::Interpolate
  EXPECT_TRUE(s.Interpolate(0.5) ==
      math::Quaternion(0.998089, 0.0315333, 0.0427683, 0.0315333));

  // ::Interpolate
  s.AddPoint(math::Quaternion(.4, .4, .4));
  EXPECT_TRUE(s.Interpolate(1, 0.2) ==
      math::Quaternion(0.978787, 0.107618, 0.137159, 0.107618));
}

/////////////////////////////////////////////////
TEST_F(MathTest, Helpers)
{
  EXPECT_EQ(12345, math::parseInt("12345"));
  EXPECT_EQ(-12345, math::parseInt("-12345"));

  EXPECT_EQ(12.345, math::parseFloat("12.345"));
  EXPECT_EQ(-12.345, math::parseFloat("-12.345"));
  EXPECT_TRUE(math::equal(123.45, math::parseFloat("1.2345e2"), 1e-2));
}

/////////////////////////////////////////////////
TEST_F(MathTest, Quaternion)
{
  {
    math::Quaternion q;
    EXPECT_TRUE(math::equal(q.w, 1.0));
    EXPECT_TRUE(math::equal(q.x, 0.0));
    EXPECT_TRUE(math::equal(q.y, 0.0));
    EXPECT_TRUE(math::equal(q.z, 0.0));
  }

  {
    math::Quaternion q(1, 2, 3, 4);
    EXPECT_TRUE(math::equal(q.w, 1.0));
    EXPECT_TRUE(math::equal(q.x, 2.0));
    EXPECT_TRUE(math::equal(q.y, 3.0));
    EXPECT_TRUE(math::equal(q.z, 4.0));
  }

  {
    math::Quaternion q(0, 1, 2);
    EXPECT_TRUE(q == math::Quaternion(math::Vector3(0, 1, 2)));
  }

  math::Quaternion q1(math::Vector3(0, 0, 1), M_PI);
  EXPECT_TRUE(math::equal(q1.x, 0.0));
  EXPECT_TRUE(math::equal(q1.y, 0.0));
  EXPECT_TRUE(math::equal(q1.z, 1.0));
  EXPECT_TRUE(math::equal(q1.w, 0.0));

  math::Quaternion q(q1);
  EXPECT_TRUE(q == q1);

  q.SetToIdentity();
  EXPECT_TRUE(math::equal(q.w, 1.0));
  EXPECT_TRUE(math::equal(q.x, 0.0));
  EXPECT_TRUE(math::equal(q.y, 0.0));
  EXPECT_TRUE(math::equal(q.z, 0.0));

  q = math::Quaternion(M_PI*0.1, M_PI*0.5, M_PI);
  EXPECT_TRUE(q == math::Quaternion(0.110616, -0.698401, 0.110616, 0.698401));

  EXPECT_TRUE(q.GetLog() ==
      math::Quaternion(0, -1.02593, 0.162491, 1.02593));

  EXPECT_TRUE(q.GetExp() ==
      math::Quaternion(0.545456, -0.588972, 0.093284, 0.588972));

  q1 = q;
  q1.w = 2.0;
  EXPECT_TRUE(q1.GetLog() ==
      math::Quaternion(0, -0.698401, 0.110616, 0.698401));

  q1.x = 0.000000001;
  q1.y = 0.0;
  q1.z = 0.0;
  q1.w = 0.0;
  EXPECT_TRUE(q1.GetExp() == math::Quaternion(1, 0, 0, 0));

  q.Invert();
  EXPECT_TRUE(q == math::Quaternion(0.110616, 0.698401, -0.110616, -0.698401));

  q.SetFromAxis(0, 1, 0, M_PI);
  EXPECT_TRUE(q == math::Quaternion(6.12303e-17, 0, 1, 0));

  q.SetFromAxis(math::Vector3(1, 0, 0), M_PI);
  EXPECT_TRUE(q == math::Quaternion(0, 1, 0, 0));

  q.Set(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(q.w, 1.0));
  EXPECT_TRUE(math::equal(q.x, 2.0));
  EXPECT_TRUE(math::equal(q.y, 3.0));
  EXPECT_TRUE(math::equal(q.z, 4.0));

  q.Normalize();
  EXPECT_TRUE(q == math::Quaternion(0.182574, 0.365148, 0.547723, 0.730297));


  EXPECT_TRUE(math::equal(q.GetRoll(), 1.4289, 1e-3));
  EXPECT_TRUE(math::equal(q.GetPitch(), -0.339837, 1e-3));
  EXPECT_TRUE(math::equal(q.GetYaw(), 2.35619, 1e-3));

  math::Vector3 axis;
  double angle;
  q.GetAsAxis(axis, angle);
  EXPECT_TRUE(axis == math::Vector3(0.371391, 0.557086, 0.742781));
  EXPECT_TRUE(math::equal(angle, 2.77438, 1e-3));

  q.Scale(0.1);
  EXPECT_TRUE(q == math::Quaternion(0.990394, 0.051354, 0.0770309, 0.102708));

  q = q + math::Quaternion(0, 1, 2);
  EXPECT_TRUE(q == math::Quaternion(1.46455, -0.352069, 0.336066, 0.841168));

  q += q;
  EXPECT_TRUE(q == math::Quaternion(2.92911, -0.704137, 0.672131, 1.68234));

  q -= math::Quaternion(.4, .2, .1);
  EXPECT_TRUE(q == math::Quaternion(1.95416, -0.896677, 0.56453, 1.65341));

  q = q - math::Quaternion(0, 1, 2);
  EXPECT_TRUE(q == math::Quaternion(1.48, -0.493254, 0.305496, 0.914947));

  q *= math::Quaternion(.4, .1, .01);
  EXPECT_TRUE(q == math::Quaternion(1.53584, -0.236801, 0.551841, 0.802979));

  q = q * 5.0;
  EXPECT_TRUE(q == math::Quaternion(7.67918, -1.184, 2.7592, 4.0149));

  gzerr << "[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n";
  gzerr << q.RotateVectorReverse(math::Vector3(1, 2, 3)) << "\n";

  EXPECT_TRUE(q.RotateVectorReverse(math::Vector3(1, 2, 3)) ==
      math::Vector3(-0.104115, 0.4975, 3.70697));

  EXPECT_TRUE(math::equal(q.Dot(math::Quaternion(.4, .2, .1)), 7.67183, 1e-3));

  EXPECT_TRUE(math::Quaternion::Squad(1.1, math::Quaternion(.1, 0, .2),
        math::Quaternion(0, .3, .4), math::Quaternion(.5, .2, 1),
        math::Quaternion(0, 0, 2), true) ==
      math::Quaternion(0.346807, -0.0511734, -0.0494723, 0.935232));

  EXPECT_TRUE(math::Quaternion::EulerToQuaternion(math::Vector3(.1, .2, .3)) ==
      math::Quaternion(0.983347, 0.0342708, 0.106021, 0.143572));

  q.Round(2);
  EXPECT_TRUE(math::equal(-1.18, q.x));
  EXPECT_TRUE(math::equal(2.76, q.y));
  EXPECT_TRUE(math::equal(4.01, q.z));
  EXPECT_TRUE(math::equal(7.68, q.w));

  q.x = q.y = q.z = q.w = 0.0;
  q.Normalize();
  EXPECT_TRUE(q == math::Quaternion());

  q.SetFromAxis(0, 0, 0, 0);
  EXPECT_TRUE(q == math::Quaternion());

  EXPECT_TRUE(math::Quaternion::EulerToQuaternion(0.1, 0.2, 0.3) ==
      math::Quaternion(0.983347, 0.0342708, 0.106021, 0.143572));

  q.x = q.y = q.z = q.w = 0.0;
  q.GetAsAxis(axis, angle);
  EXPECT_TRUE(axis == math::Vector3(1, 0, 0));
  EXPECT_TRUE(math::equal(angle, 0.0, 1e-3));
  {
    // simple 180 rotation about yaw, should result in x and y flipping signs
    q = math::Quaternion(0, 0, M_PI);
    math::Vector3 v = math::Vector3(1, 2, 3);
    math::Vector3 r1 = q.RotateVector(v);
    math::Vector3 r2 = q.RotateVectorReverse(v);
    gzdbg << "[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n";
    gzdbg << " forward turns [" << v << "] to [" << r1 << "]\n";
    gzdbg << " reverse turns [" << v << "] to [" << r2 << "]\n";
    EXPECT_TRUE(r1 == math::Vector3(-1, -2, 3));
    EXPECT_TRUE(r2 == math::Vector3(-1, -2, 3));
  }

  {
    // simple  90 rotation about yaw, should map x to y, y to -x
    // simple -90 rotation about yaw, should map x to -y, y to x
    q = math::Quaternion(0, 0, 0.5*M_PI);
    math::Vector3 v = math::Vector3(1, 2, 3);
    math::Vector3 r1 = q.RotateVector(v);
    math::Vector3 r2 = q.RotateVectorReverse(v);
    gzdbg << "[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n";
    gzdbg << " forward turns [" << v << "] to [" << r1 << "]\n";
    gzdbg << " reverse turns [" << v << "] to [" << r2 << "]\n";
    gzdbg << " x axis [" << q.GetXAxis() << "]\n";
    gzdbg << " y axis [" << q.GetYAxis() << "]\n";
    gzdbg << " z axis [" << q.GetZAxis() << "]\n";
    EXPECT_TRUE(r1 == math::Vector3(-2, 1, 3));
    EXPECT_TRUE(r2 == math::Vector3(2, -1, 3));
    EXPECT_TRUE(q.GetInverse().GetXAxis() == math::Vector3(0, -1, 0));
    EXPECT_TRUE(q.GetInverse().GetYAxis() == math::Vector3(1, 0, 0));
    EXPECT_TRUE(q.GetInverse().GetZAxis() == math::Vector3(0, 0, 1));
  }

  {
    // now try a harder case (axis[1,2,3], rotation[0.3*pi])
    // verified with octave
    q.SetFromAxis(math::Vector3(1, 2, 3), 0.3*M_PI);
    gzdbg << "[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n";
    gzdbg << " x [" << q.GetInverse().GetXAxis() << "]\n";
    gzdbg << " y [" << q.GetInverse().GetYAxis() << "]\n";
    gzdbg << " z [" << q.GetInverse().GetZAxis() << "]\n";
    EXPECT_TRUE(q.GetInverse().GetXAxis() ==
                math::Vector3(0.617229, -0.589769, 0.520770));
    EXPECT_TRUE(q.GetInverse().GetYAxis() ==
                math::Vector3(0.707544, 0.705561, -0.039555));
    EXPECT_TRUE(q.GetInverse().GetZAxis() ==
                math::Vector3(-0.344106, 0.392882, 0.852780));

    // rotate about the axis of rotation should not change axis
    math::Vector3 v = math::Vector3(1, 2, 3);
    math::Vector3 r1 = q.RotateVector(v);
    math::Vector3 r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3(1, 2, 3));
    EXPECT_TRUE(r2 == math::Vector3(1, 2, 3));

    // rotate unit vectors
    v = math::Vector3(0, 0, 1);
    r1 = q.RotateVector(v);
    r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3(0.520770, -0.039555, 0.852780));
    EXPECT_TRUE(r2 == math::Vector3(-0.34411, 0.39288, 0.85278));
    v = math::Vector3(0, 1, 0);
    r1 = q.RotateVector(v);
    r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3(-0.58977, 0.70556, 0.39288));
    EXPECT_TRUE(r2 == math::Vector3(0.707544, 0.705561, -0.039555));
    v = math::Vector3(1, 0, 0);
    r1 = q.RotateVector(v);
    r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3(0.61723, 0.70754, -0.34411));
    EXPECT_TRUE(r2 == math::Vector3(0.61723, -0.58977, 0.52077));

    EXPECT_TRUE(-q == math::Quaternion(-0.891007, -0.121334,
                                       -0.242668, -0.364002));

    EXPECT_TRUE(q.GetAsMatrix3() == math::Matrix3(
                0.617229, -0.589769, 0.52077,
                0.707544, 0.705561, -0.0395554,
                -0.344106, 0.392882, 0.85278));

    EXPECT_TRUE(q.GetAsMatrix4() == math::Matrix4(
                0.617229, -0.589769, 0.52077, 0,
                0.707544, 0.705561, -0.0395554, 0,
                -0.344106, 0.392882, 0.85278, 0,
                0, 0, 0, 1));
  }
}

TEST_F(MathTest, Pose)
{
  {
    // test hypothesis that if
    // A is the transform from O to P specified in frame O
    // B is the transform from P to Q specified in frame P
    // then, B + A is the transform from O to Q specified in frame O
    math::Pose A(math::Vector3(1, 0, 0), math::Quaternion(0, 0, M_PI/4.0));
    math::Pose B(math::Vector3(1, 0, 0), math::Quaternion(0, 0, M_PI/2.0));
    EXPECT_TRUE(math::equal((B + A).pos.x, 1.0 + 1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((B + A).pos.y,       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((B + A).pos.z,               0.0));
    EXPECT_TRUE(math::equal((B + A).rot.GetAsEuler().x,  0.0));
    EXPECT_TRUE(math::equal((B + A).rot.GetAsEuler().y,  0.0));
    EXPECT_TRUE(math::equal((B + A).rot.GetAsEuler().z, 3.0*M_PI/4.0));
  }
  {
    // If:
    // A is the transform from O to P in frame O
    // B is the transform from O to Q in frame O
    // then -A is transform from P to O specified in frame P
    math::Pose A(math::Vector3(1, 0, 0), math::Quaternion(0, 0, M_PI/4.0));
    EXPECT_TRUE(math::equal((math::Pose() - A).pos.x,      -1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((math::Pose() - A).pos.y,       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((math::Pose() - A).pos.z,               0.0));
    EXPECT_TRUE(math::equal((math::Pose() - A).rot.GetAsEuler().x,  0.0));
    EXPECT_TRUE(math::equal((math::Pose() - A).rot.GetAsEuler().y,  0.0));
    EXPECT_TRUE(math::equal((math::Pose() - A).rot.GetAsEuler().z, -M_PI/4.0));

    // test negation operator
    EXPECT_TRUE(math::equal((-A).pos.x,      -1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((-A).pos.y,       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((-A).pos.z,               0.0));
    EXPECT_TRUE(math::equal((-A).rot.GetAsEuler().x,  0.0));
    EXPECT_TRUE(math::equal((-A).rot.GetAsEuler().y,  0.0));
    EXPECT_TRUE(math::equal((-A).rot.GetAsEuler().z, -M_PI/4.0));
  }
  {
    // If:
    // A is the transform from O to P in frame O
    // B is the transform from O to Q in frame O
    // B - A is the transform from P to Q in frame P
    math::Pose A(math::Vector3(1, 0, 0), math::Quaternion(0, 0, M_PI/4.0));
    math::Pose B(math::Vector3(1, 1, 0), math::Quaternion(0, 0, M_PI/2.0));
    EXPECT_TRUE(math::equal((B - A).pos.x,       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((B - A).pos.y,       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((B - A).pos.z,               0.0));
    EXPECT_TRUE(math::equal((B - A).rot.GetAsEuler().x,  0.0));
    EXPECT_TRUE(math::equal((B - A).rot.GetAsEuler().y,  0.0));
    EXPECT_TRUE(math::equal((B - A).rot.GetAsEuler().z, M_PI/4.0));
  }
  {
    math::Pose pose;
    EXPECT_TRUE(pose.pos == math::Vector3(0, 0, 0));
    EXPECT_TRUE(pose.rot == math::Quaternion(0, 0, 0));
  }

  math::Pose pose(math::Vector3(1, 2, 3), math::Quaternion(.1, .2, .3));
  EXPECT_TRUE(pose.pos == math::Vector3(1, 2, 3));
  EXPECT_TRUE(pose.rot == math::Quaternion(.1, .2, .3));

  math::Pose pose1(pose);
  EXPECT_TRUE(pose1 == pose);

  pose.Set(math::Vector3(2, 3, 4), math::Quaternion(.3, .4, .5));
  EXPECT_TRUE(pose.pos == math::Vector3(2, 3, 4));
  EXPECT_TRUE(pose.rot == math::Quaternion(.3, .4, .5));
  EXPECT_TRUE(pose.IsFinite());

  pose1 = pose.GetInverse();
  EXPECT_TRUE(pose1.pos == math::Vector3(-1.38368, -3.05541, -4.21306));
  EXPECT_TRUE(pose1.rot ==
      math::Quaternion(0.946281, -0.0933066, -0.226566, -0.210984));

  pose = math::Pose(1, 2, 3, .1, .2, .3) + math::Pose(4, 5, 6, .4, .5, .6);
  EXPECT_TRUE(pose ==
      math::Pose(5.74534, 7.01053, 8.62899, 0.675732, 0.535753, 1.01174));

  pose += pose;
  EXPECT_TRUE(pose ==
      math::Pose(11.314, 16.0487, 15.2559, 1.49463, 0.184295, 2.13932));

  pose -= math::Pose(pose);
  EXPECT_TRUE(pose ==
      math::Pose(0, 0, 0, 0, 0, 0));

  pose.pos.Set(5, 6, 7);
  pose.rot.SetFromEuler(math::Vector3(.4, .6, 0));

  EXPECT_TRUE(pose.CoordPositionAdd(math::Vector3(1, 2, 3)) ==
      math::Vector3(7.82531, 6.67387, 9.35871));

  EXPECT_TRUE(pose.CoordPositionAdd(pose1) ==
      math::Vector3(2.58141, 2.4262, 3.8013));
  EXPECT_TRUE(pose.CoordRotationAdd(math::Quaternion(0.1, 0, 0.2)) ==
      math::Quaternion(0.520975, 0.596586, 0.268194));
  EXPECT_TRUE(pose.CoordPoseSolve(pose1) ==
      math::Pose(-0.130957, -11.552, -10.2329,
                 -0.462955, -1.15624, -0.00158047));

  EXPECT_TRUE(pose.RotatePositionAboutOrigin(math::Quaternion(0.1, 0, 0.2)) ==
      math::Pose(6.09235, 5.56147, 6.47714, 0.4, 0.6, 0));

  pose.Reset();
  EXPECT_TRUE(pose.pos == math::Vector3(0, 0, 0));
  EXPECT_TRUE(pose.rot == math::Quaternion(0, 0, 0));
}

TEST_F(MathTest, Plane)
{
  {
    math::Plane plane;
    EXPECT_TRUE(math::equal(plane.d, 0.0));
    EXPECT_TRUE(plane.normal == math::Vector3());
    EXPECT_TRUE(plane.size == math::Vector2d(0, 0));
  }

  {
    math::Plane plane(math::Vector3(0, 0, 1), math::Vector2d(2, 3), 2.0);
    EXPECT_TRUE(math::equal(plane.d, 2.0));
    EXPECT_TRUE(plane.normal == math::Vector3(0, 0, 1));
    EXPECT_TRUE(plane.size == math::Vector2d(2, 3));

    EXPECT_EQ(-1, plane.Distance(math::Vector3(0, 0, 1),
                                 math::Vector3(0, 0, -1)));

    plane.Set(math::Vector3(1, 0, 0), math::Vector2d(1, 1), 1.0);
    EXPECT_TRUE(math::equal(plane.d, 1.0));
    EXPECT_TRUE(plane.normal == math::Vector3(1, 0, 0));
    EXPECT_TRUE(plane.size == math::Vector2d(1, 1));

    plane = math::Plane(math::Vector3(0, 1, 0), math::Vector2d(4, 4), 5.0);
    EXPECT_TRUE(math::equal(plane.d, 5.0));
    EXPECT_TRUE(plane.normal == math::Vector3(0, 1, 0));
    EXPECT_TRUE(plane.size == math::Vector2d(4, 4));
  }
}

TEST_F(MathTest, Matrix3)
{
  {
    math::Matrix3 matrix;
    EXPECT_TRUE(matrix == math::Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0));
  }

  {
    math::Matrix3 matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
    EXPECT_TRUE(matrix == math::Matrix3(1, 2, 3, 4, 5, 6, 7, 8, 9));

    math::Matrix3 matrix1(matrix);
    EXPECT_TRUE(matrix1 == math::Matrix3(1, 2, 3, 4, 5, 6, 7, 8, 9));
  }

  math::Matrix3 matrix;
  matrix.SetFromAxes(math::Vector3(1, 1, 1), math::Vector3(2, 2, 2),
                     math::Vector3(3, 3, 3));
  EXPECT_TRUE(matrix == math::Matrix3(1, 2, 3, 1, 2, 3, 1, 2, 3));

  matrix.SetFromAxis(math::Vector3(1, 1, 1), M_PI);
  EXPECT_TRUE(matrix == math::Matrix3(1, 2, 2, 2, 1, 2, 2, 2, 1));

  matrix.SetCol(0, math::Vector3(3, 4, 5));
  EXPECT_TRUE(matrix == math::Matrix3(3, 2, 2, 4, 1, 2, 5, 2, 1));

  EXPECT_THROW(matrix.SetCol(3, math::Vector3(1, 1, 1)), std::string);
}

TEST_F(MathTest, Angle)
{
  math::Angle angle1;
  EXPECT_TRUE(math::equal(0.0, angle1.Radian()));

  angle1.SetFromDegree(180.0);
  EXPECT_TRUE(angle1 == M_PI);

  angle1 = math::Angle(0.1) - math::Angle(0.3);
  EXPECT_TRUE(angle1 == -0.2);

  math::Angle angle(0.5);
  EXPECT_TRUE(math::equal(0.5, angle.Radian()));

  angle.SetFromRadian(M_PI);
  EXPECT_TRUE(math::equal(GZ_RTOD(M_PI), angle.Degree()));

  angle.Normalize();
  EXPECT_TRUE(math::equal(GZ_RTOD(M_PI), angle.Degree()));

  angle = math::Angle(0.1) + math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.3, angle.Radian()));

  angle = math::Angle(0.1) * math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.02, angle.Radian()));

  angle = math::Angle(0.1) / math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.5, angle.Radian()));

  angle -= math::Angle(0.1);
  EXPECT_TRUE(math::equal(0.4, angle.Radian()));

  angle += math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.6, angle.Radian()));

  angle *= math::Angle(0.5);
  EXPECT_TRUE(math::equal(0.3, angle.Radian()));

  angle /= math::Angle(0.1);
  EXPECT_TRUE(math::equal(3.0, angle.Radian()));
  EXPECT_TRUE(angle == math::Angle(3));
  EXPECT_TRUE(angle != math::Angle(2));
  EXPECT_TRUE(angle < math::Angle(4));
  EXPECT_TRUE(angle <= math::Angle(3));
  EXPECT_TRUE(angle > math::Angle(2));
  EXPECT_TRUE(angle >= math::Angle(3));
}

TEST_F(MathTest, Random)
{
  double d;
  int i;
  // TODO: implement a proper random number generator test

  d = gazebo::math::Rand::GetDblUniform(1, 2);
  EXPECT_LE(d, 2);
  EXPECT_GE(d, 1);

  d = math::Rand::GetDblNormal(2, 3);
  i = math::Rand::GetIntUniform(1, 2);
  EXPECT_LE(i, 2);
  EXPECT_GE(i, 1);

  i = math::Rand::GetIntNormal(2, 3);

  // Test setting the random number seed
  {
    math::Rand::SetSeed(1001);

    d = math::Rand::GetDblNormal(2, 3);
    EXPECT_TRUE(math::equal(d, 0.985827));
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
