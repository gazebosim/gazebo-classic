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
  // Do not forget to update 'precomputedSHA1' if you modify this text
  std::string s;
  s = "Marty McFly: Wait a minute, Doc. Ah... Are you telling me that you"
      " built a time machine... out of a DeLorean?\n"
      "Dr. Emmett Brown: The way I see it, if you're gonna build a time"
      " machine into a car, why not do it with some style?";

  // Compute the SHA1 of the file and compare it with the precomputed SHA1
  std::string precomputedSHA1 = "a370ddc4d61d936b2bb40f98bae061dc15fd8923";
  std::string computedSHA1 = common::get_sha1(s.data(), s.size());
  EXPECT_EQ(precomputedSHA1, computedSHA1);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
