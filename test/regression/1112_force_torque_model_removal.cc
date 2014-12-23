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

using namespace gazebo;

class Issue1112Test : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test for issue #1112
TEST_F(Issue1112Test, Reset)
{
  Load("worlds/force_torque_model_removal_test.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Remove everything
  world->Clear();

  // Wait some bit of time since World::Clear is not immediate.
  int sleepCount = 20;
  int sleep = 0;
  while (world->GetModelCount() > 0u && sleep < sleepCount)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // Expecting that the world is empty (and the simulation did not crash)
  EXPECT_EQ(world->GetModelCount(), 0u);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
