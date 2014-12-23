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

#include "ServerFixture.hh"
#include "gazebo/math/Rand.hh"

using namespace gazebo;

class Issue876Test : public ServerFixture
{
};


/////////////////////////////////////////////////
// \brief Test for issue #876
TEST_F(Issue876Test, Reset)
{
  Load("worlds/empty.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  math::Rand::SetSeed(math::Rand::GetSeed());

  int sampleCount = 500;

  std::vector<int> num;
  for (int i = 0; i < sampleCount; ++i)
    num.push_back(math::Rand::GetIntUniform(-10, 10));

  for (int j = 0; j < 1000; ++j)
  {
    world->Reset();

    std::vector<int> numReset;
    for (int i = 0; i < sampleCount; ++i)
      numReset.push_back(math::Rand::GetIntUniform(-10, 10));

    // Using ASSERT_EQ to prevent spamming of similar errors.
    for (int i = 0; i < sampleCount; ++i)
      ASSERT_EQ(num[i], numReset[i]);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
