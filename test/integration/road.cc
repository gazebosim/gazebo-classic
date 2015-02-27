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

#include <map>
#include <string>
#include <vector>

#include "ServerFixture.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/physics.hh"
#include "SimplePendulumIntegrator.hh"
#include "gazebo/msgs/msgs.hh"
#include "helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class RoadTextureTest : public ServerFixture
{
  public: void TexturedWorld();
};

void RoadTextureTest::TexturedWorld()
{
  // Load the sample world
  Load("worlds/road_textures.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  // simulate a few steps
  int steps = 20;
  world->Step(steps);
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  t = world->GetSimTime().Double();
  EXPECT_GT(t, 0.99*dt*static_cast<double>(steps+1));
}

TEST_F(RoadTextureTest, TexturedWorld)
{
  TexturedWorld();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
