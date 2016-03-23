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

#include <sys/time.h>
#include <gtest/gtest.h>

#include "gazebo/common/SemanticVersion.hh"
#include "test/util.hh"

using namespace gazebo;
using namespace common;

class SemVerTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(SemVerTest, Operators)
{
  SemanticVersion a("0.1.0");
  SemanticVersion b("1.0.0");
  SemanticVersion c("1.0.0");
  SemanticVersion c2("1.0.2");

  // check that the short form is the same as the long one
  SemanticVersion aa("0.1");
  EXPECT_EQ(aa.Version(), "0.1.0");

  // check second constructor
  SemanticVersion c2b(1, 0, 2);
  EXPECT_TRUE(c2 == c2b);

  EXPECT_TRUE(a < b);
  EXPECT_TRUE(a <= b);

  // equality
  EXPECT_TRUE(a != b);
  EXPECT_TRUE(b == c);
  EXPECT_FALSE(a == b);
  EXPECT_FALSE(a != a);

  // other operators
  EXPECT_TRUE(b <= c);
  EXPECT_TRUE(b >= c);
  EXPECT_TRUE(c2 > c);
  EXPECT_TRUE(b == b);
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

