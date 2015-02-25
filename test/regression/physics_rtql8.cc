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
#include "physics/physics.hh"
#include "SimplePendulumIntegrator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;
class PhysicsTest : public ServerFixture
{
  public: void EmptyWorld_JS(std::string _worldFile);

  public: void EmptyWorld(const std::string &_physicsEngine);
  public: void SpawnDrop(const std::string &_physicsEngine);
  public: void SpawnDropCoGOffset(const std::string &_physicsEngine);
  public: void RevoluteJoint(const std::string &_physicsEngine);
  public: void SimplePendulum(const std::string &_physicsEngine);
  public: void CollisionFiltering(const std::string &_physicsEngine);
};

void PhysicsTest::EmptyWorld_JS(std::string _worldFile)
{
  // Load an empty world
  Load(_worldFile, true);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  // simulate a couple seconds
  world->StepWorld(2000);
  double t = world->GetSimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  Unload();
}

#ifdef HAVE_RTQL8
TEST_F(PhysicsTest, RTQL8_FIRST_TEST)
{
  EmptyWorld_JS("worlds/empty.world");
  //EmptyWorld_JS("worlds/rtql8/simple_pendulum.world");
}
#endif // HAVE_RTQL8

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
