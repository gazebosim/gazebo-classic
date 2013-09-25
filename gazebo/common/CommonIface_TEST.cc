/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <stdlib.h>
#include <fstream>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "gazebo/common/CommonIface.hh"

using namespace gazebo;

/////////////////////////////////////////////////
/// \brief Test CommonIface::GetSHA1
TEST(CommonIface_TEST, GetSHA1)
{
  // Do not forget to update 'precomputedSHA1' if you modify this vector
  std::vector<float> v;
  for (int i = 0; i < 100; ++i)
    v.push_back(i);

  // Compute the SHA1 of the vector
  std::string precomputedSHA1 = "913283ec8502ba1423d38a7ea62cb8e492e87b23";
  std::string computedSHA1 = common::get_sha1(v);
  EXPECT_EQ(precomputedSHA1, computedSHA1);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
