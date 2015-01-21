/*
 * Copyright 2013-2015 Open Source Robotics Foundation
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
class GzWorld : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test world pause.
TEST_F(GzWorld, Pause)
{
  Load("worlds/empty_test.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  EXPECT_FALSE(world->IsPaused());

  // Pause
  custom_exec("gz world -p 1");

  EXPECT_TRUE(world->IsPaused());

  // Run
  custom_exec("gz world -p 0");

  EXPECT_FALSE(world->IsPaused());
}

/////////////////////////////////////////////////
// \brief Test world step.
TEST_F(GzWorld, Step)
{
  Load("worlds/empty_test.world", true);

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  EXPECT_TRUE(world->IsPaused());

  EXPECT_EQ(world->GetIterations(), 0u);

  // Step the world one iteration.
  custom_exec("gz world -s 1");

  EXPECT_EQ(world->GetIterations(), 1u);

  EXPECT_TRUE(world->IsPaused());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
