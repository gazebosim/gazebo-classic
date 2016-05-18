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

#include <gtest/gtest.h>
#include "gazebo/physics/plugin/PluginTypes.hh"
#include "gazebo/physics/plugin/PluginJoint.hh"
#include "gazebo/physics/physics.hh"
#include "test/ServerFixture.hh"

#define TOL 1e-6
using namespace gazebo;

class PluginJoint_TEST : public ServerFixture
{
};

////////////////////////////////////////////////////////////////////////
// Test multi-axis universal joints
// with implicit (cfm) damping
////////////////////////////////////////////////////////////////////////
TEST_F(PluginJoint_TEST, PluginJointTest)
{
  // Load our force torque test world
  Load("worlds/empty.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), "plugin");

  physics->SetGravity(math::Vector3(0, 0, -50));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzdbg << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_GT(t, 0);
  gzdbg << "t after one step : " << t << "\n";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
