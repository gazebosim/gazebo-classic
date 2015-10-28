/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "test/util.hh"

using namespace gazebo;

class HelpersTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
// Test a few function in Helpers
TEST_F(HelpersTest, Helpers)
{
  EXPECT_EQ(12345, math::parseInt("12345"));
  EXPECT_EQ(-12345, math::parseInt("-12345"));

  EXPECT_FLOAT_EQ(12.345, math::parseFloat("12.345"));
  EXPECT_FLOAT_EQ(-12.345, math::parseFloat("-12.345"));
  EXPECT_TRUE(math::equal(123.45, math::parseFloat("1.2345e2"), 1e-2));

  EXPECT_EQ(1u, math::roundUpPowerOfTwo(0));
  EXPECT_EQ(1u, math::roundUpPowerOfTwo(1));
  EXPECT_EQ(2u, math::roundUpPowerOfTwo(2));
  EXPECT_EQ(2048u, math::roundUpPowerOfTwo(1025));
}

/////////////////////////////////////////////////
// Test Helpers::fixnan functions
TEST_F(HelpersTest, FixNaN)
{
  EXPECT_DOUBLE_EQ(math::fixnan(1.0 / 0.0), 0.0);
  EXPECT_DOUBLE_EQ(math::fixnan(-1.0 / 0.0), 0.0);
  EXPECT_DOUBLE_EQ(math::fixnan(0.0 / 0.0), 0.0);

  EXPECT_DOUBLE_EQ(math::fixnan(42.0), 42.0);
  EXPECT_DOUBLE_EQ(math::fixnan(-42.0), -42.0);

  EXPECT_FLOAT_EQ(math::fixnan(1.0f / 0.0f), 0.0f);
  EXPECT_FLOAT_EQ(math::fixnan(-1.0f / 0.0f), 0.0f);
  EXPECT_FLOAT_EQ(math::fixnan(0.0f / 0.0f), 0.0f);

  EXPECT_FLOAT_EQ(math::fixnan(42.0f), 42.0f);
  EXPECT_FLOAT_EQ(math::fixnan(-42.0f), -42.0f);
}
