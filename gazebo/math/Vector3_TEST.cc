/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "gazebo/math/Vector3.hh"
#include "test/util.hh"

using namespace gazebo;

class Vector3Test : public gazebo::testing::AutoLogFixture { };

TEST_F(Vector3Test, Vector3)
{
  math::Vector3 v;

  // ::Distance, ::GetLEngth()
  v.Set(1, 2, 3);
  EXPECT_FLOAT_EQ(v.GetLength(), v.Distance(math::Vector3(0, 0, 0)));

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
  EXPECT_DOUBLE_EQ(1.0, v.GetDistToLine(math::Vector3(1, -1, 0),
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
  EXPECT_DOUBLE_EQ(v[0], 4);
  EXPECT_DOUBLE_EQ(v[1], 3);
  EXPECT_DOUBLE_EQ(v[2], 2);

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

  // Test the static defines.
  EXPECT_TRUE(math::Vector3::Zero == math::Vector3(0, 0, 0));
  EXPECT_TRUE(math::Vector3::One == math::Vector3(1, 1, 1));
  EXPECT_TRUE(math::Vector3::UnitX == math::Vector3(1, 0, 0));
  EXPECT_TRUE(math::Vector3::UnitY == math::Vector3(0, 1, 0));
  EXPECT_TRUE(math::Vector3::UnitZ == math::Vector3(0, 0, 1));
}
