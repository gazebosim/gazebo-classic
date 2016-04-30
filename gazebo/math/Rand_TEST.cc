/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/math/Helpers.hh"
#include "gazebo/math/Rand.hh"

using namespace gazebo;

class RandTest : public ::testing::Test { };

//////////////////////////////////////////////////
TEST_F(RandTest, Rand)
{
  double d;
  int i;
  // TODO: implement a proper random number generator test

  d = gazebo::math::Rand::GetDblUniform(1, 2);
  EXPECT_LE(d, 2);
  EXPECT_GE(d, 1);

  i = math::Rand::GetIntUniform(1, 2);
  EXPECT_LE(i, 2);
  EXPECT_GE(i, 1);

  i = math::Rand::GetIntNormal(2, 3);
  EXPECT_LE(i, GZ_INT32_MAX);
  EXPECT_GE(i, -GZ_INT32_MAX);

  // Test setting the random number seed
  {
    // Set the seed and get two numbers
    math::Rand::SetSeed(1001);
    d = math::Rand::GetDblNormal(2, 3);
    i = math::Rand::GetIntNormal(2, 3);

    // Get two more numbers
    double d1 = math::Rand::GetDblNormal(2, 3);
    int i1 = math::Rand::GetIntNormal(2, 3);

    // and make sure they're different
    EXPECT_GT(std::abs(d - d1), 0);
    EXPECT_NE(i, i1);

    // Reset the seed to the original value
    math::Rand::SetSeed(1001);

    // Get two numbers again
    double d2 = math::Rand::GetDblNormal(2, 3);
    int i2 = math::Rand::GetIntNormal(2, 3);

    // and make sure they match the first numbers
    EXPECT_DOUBLE_EQ(d, d2);
    EXPECT_EQ(i, i2);
  }
}

//////////////////////////////////////////////////
TEST_F(RandTest, SetSeed)
{
  int N = 10;
  std::vector<int> first;
  std::vector<int> second;

  for (int i = 0; i < N; ++i)
  {
    math::Rand::SetSeed(i);
    first.push_back(math::Rand::GetIntUniform(-10, 10));
    second.push_back(math::Rand::GetIntUniform(-10, 10));
  }

  for (int i = 0; i < N; ++i)
  {
    math::Rand::SetSeed(i);
    EXPECT_EQ(first[i], math::Rand::GetIntUniform(-10, 10));
    EXPECT_EQ(second[i], math::Rand::GetIntUniform(-10, 10));
  }
}
