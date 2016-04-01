/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include <string>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class SwarmTest : public ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  public: void FlockingWorld(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// FlockingWorld:
// Load a world with 1001 robots
//////////////////////////////////////////////////////////////////////////
void SwarmTest::FlockingWorld(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/flocking.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  EXPECT_EQ(world->GetModels().size(), 1003u);
}

TEST_P(SwarmTest, FlockingWorld)
{
  FlockingWorld(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, SwarmTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
