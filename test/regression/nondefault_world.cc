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
#include "ServerFixture.hh"

using namespace gazebo;
class NonDefaultWorld : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(NonDefaultWorld, Load)
{
  math::Pose setPose, testPose;
  Load("worlds/empty_different_name.world");
  SetPause(false);

  int i = 0;

  // Wait for the entity to spawn
  while (!HasEntity("ground_plane") && i < 10)
  {
    common::Time::MSleep(10);
    ++i;
  }

  EXPECT_TRUE(HasEntity("ground_plane"));
  EXPECT_LT(i, 10);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
