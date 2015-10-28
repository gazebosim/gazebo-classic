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

#include <stdlib.h>
#include <gtest/gtest.h>
#include <string>
#include <sstream>

#include "gazebo/common/CommonIface.hh"
#include "test/util.hh"

using namespace gazebo;

class CommonIface_TEST : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
/// \brief Test CommonIface::GetSHA1
TEST_F(CommonIface_TEST, GetSHA1)
{
  // Do not forget to update 'precomputedSHA1' if you modify the SHA1 input.
  std::string precomputedSHA1;
  std::string computedSHA1;
  std::string s;

  // Compute the SHA1 of the vector
  std::vector<float> v;
  for (int i = 0; i < 100; ++i)
    v.push_back(i);

  computedSHA1 = common::get_sha1<std::vector<float> >(v);
  precomputedSHA1 = "913283ec8502ba1423d38a7ea62cb8e492e87b23";
  EXPECT_EQ(precomputedSHA1, computedSHA1);

  // Compute the SHA1 of a string
  s = "Marty McFly: Wait a minute, Doc. Ah... Are you telling me that you"
      " built a time machine... out of a DeLorean?\n"
      "Dr. Emmett Brown: The way I see it, if you're gonna build a time"
      " machine into a car, why not do it with some style?";
  computedSHA1 = common::get_sha1<std::string>(s);
  precomputedSHA1 = "a370ddc4d61d936b2bb40f98bae061dc15fd8923";
  EXPECT_EQ(precomputedSHA1, computedSHA1);

  // Compute the SHA1 of an empty string
  s = "";
  computedSHA1 = common::get_sha1<std::string>(s);
  precomputedSHA1 = "da39a3ee5e6b4b0d3255bfef95601890afd80709";
  EXPECT_EQ(precomputedSHA1, computedSHA1);

  // Compute a bunch of SHA1's to verify consistent length
  for (unsigned i = 0; i < 100; ++i)
  {
    std::stringstream stream;
    stream << i << '\n';
    std::string sha = common::get_sha1<std::string>(stream.str());
    EXPECT_EQ(sha.length(), 40u);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
