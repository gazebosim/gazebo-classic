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
#include <string.h>

#include "gazebo/physics/physics.hh"
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;

typedef std::tr1::tuple<const char *, int> string_uint;

class WorldResetTest : public ServerFixture,
                       public ::testing::WithParamInterface<string_uint>
{
  public: void Empty(const std::string &_physicsEngine, int _resets);
};

/////////////////////////////////////////////////
void WorldResetTest::Empty(const std::string &_physicsEngine,
                                    int _resets)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  double dt = physics->GetMaxStepSize();
  unsigned int steps = 250;

  // Step forward, verify time increasing
  world->StepWorld(steps);
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
    world->StepWorld(steps);
    simTime = world->GetSimTime().Double();
    EXPECT_NEAR(simTime, dt*steps, dt);
  }
}

/////////////////////////////////////////////////
TEST_P(WorldResetTest, Empty)
{
  std::string physics = std::tr1::get<0>(GetParam());
  int resets = std::tr1::get<1>(GetParam());
  gzdbg << "Physics engine [" << physics << "], "
        << "reset count [" << resets << "]"
        << std::endl;
  Empty(physics, resets);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldResetTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Range(1, 4)));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
