/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

typedef std::tr1::tuple<const char *, const char *, int> string2_int;

class WorldResetTest : public ServerFixture,
                       public ::testing::WithParamInterface<string2_int>
{
  public: void WorldName(const std::string &_physicsEngine,
                         const std::string &_world, int _resets);
};

/////////////////////////////////////////////////
void WorldResetTest::WorldName(const std::string &_physicsEngine,
                               const std::string &_world, int _resets)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray sensor in PR2, "
          << "Please see issue #911.\n";
    return;
  }

  Load(_world, true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  double dt = physics->GetMaxStepSize();
  unsigned int steps = 250;

  // Step forward, verify time increasing
  world->Step(steps);
  double simTime = world->GetSimTime().Double();
  EXPECT_NEAR(simTime, dt*steps, dt);

  // Reset world repeatedly
  for (int i = 0; i < _resets; ++i)
  {
    // Reset world, verify time == 0
    world->Reset();
    simTime = world->GetSimTime().Double();
    EXPECT_NEAR(simTime, 0.0, dt);

    // Step forward, verify time increasing
    world->Step(steps);
    simTime = world->GetSimTime().Double();
    EXPECT_NEAR(simTime, dt*steps, dt);
  }
}

/////////////////////////////////////////////////
TEST_P(WorldResetTest, WorldName)
{
  std::string physics = std::tr1::get<0>(GetParam());
  std::string worldName = std::tr1::get<1>(GetParam());
  int resets = std::tr1::get<2>(GetParam());
  gzdbg << "Physics engine [" << physics << "] "
        << "world name [" << worldName << "] "
        << "reset count [" << resets << "]"
        << std::endl;
  WorldName(physics, worldName, resets);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldResetTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("worlds/empty.world"
                  , "worlds/pr2.world"),
  ::testing::Range(1, 3)));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
