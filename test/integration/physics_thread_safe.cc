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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PhysicsThreadSafeTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  public: void LinkGet(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsThreadSafeTest::LinkGet(const std::string &_physicsEngine)
{
  Load("worlds/revolute_joint_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Unthrottle the update rate
  physics->SetRealTimeUpdateRate(0);

  std::string modelName = "pendulum_0deg";
  std::string linkName = "lower_link";

  physics::ModelPtr model = world->GetModel(modelName);
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink(linkName);
  ASSERT_TRUE(link != NULL);

  // Start the simulation
  world->SetPaused(false);

  // Run for 5 seconds of sim time
  while (world->GetSimTime().sec < 5)
  {
    // Call these functions repeatedly
    // Test passes if it doesn't abort early
    math::Vector3 vel = link->GetWorldLinearVel();
    vel += link->GetWorldLinearVel(math::Vector3());
    vel += link->GetWorldLinearVel(math::Vector3(), math::Quaternion());
    vel += link->GetWorldCoGLinearVel();
    vel += link->GetWorldAngularVel();
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsThreadSafeTest, LinkGet)
{
  LinkGet(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsThreadSafeTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
