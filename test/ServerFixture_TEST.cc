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

#include "ServerFixture.hh"

using namespace gazebo;
class ServerFixtureTest : public ServerFixture
{
  public: void LoadEmptyOfType(const std::string _physicsType);
};

////////////////////////////////////////////////////////////////////////
// LoadPaused:
// Verify that ServerFixture can load world in paused state
// Gazebo issue #334
////////////////////////////////////////////////////////////////////////
TEST_F(ServerFixtureTest, LoadPaused)
{
  // Note the second argument of Load sets the pause state
  Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  gzdbg << "Check IsPaused with no delay\n";
  EXPECT_TRUE(world->IsPaused());

  common::Time::MSleep(100);
  gzdbg << "Check IsPaused with 100 ms delay\n";
  EXPECT_TRUE(world->IsPaused());

  common::Time::MSleep(900);
  gzdbg << "Check IsPaused with 1000 ms delay\n";
  EXPECT_TRUE(world->IsPaused());
}

////////////////////////////////////////////////////////////////////////
// LoadEmptyOfType:
// Verify that ServerFixture can load empty world with different types
// of physics engines (issue #486)
////////////////////////////////////////////////////////////////////////
void ServerFixtureTest::LoadEmptyOfType(const std::string _physicsType)
{
  // Note the second argument of Load sets the pause state
  Load("worlds/empty.world", true, _physicsType);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  EXPECT_EQ(physics->GetType(), _physicsType);
}

TEST_F(ServerFixtureTest, LoadODE)
{
  LoadEmptyOfType("ode");
}

#ifdef HAVE_BULLET
TEST_F(ServerFixtureTest, LoadBullet)
{
  LoadEmptyOfType("bullet");
}
#endif  // HAVE_BULLET

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
