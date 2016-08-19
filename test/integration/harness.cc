/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <cmath>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "scans_cmp.h"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
class Harness : public ServerFixture,
                public testing::WithParamInterface<const char*>
{
  /// \brief Lower, stop, then raise harness.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void LowerStopRaise(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
void Harness::LowerStopRaise(const std::string &_physicsEngine)
{
  Load("worlds/harness.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world();
  ASSERT_NE(world , nullptr);

  auto physics = world->GetPhysicsEngine();
  ASSERT_NE(physics, nullptr);
  double dt = physics->GetMaxStepSize();
  EXPECT_NEAR(dt, 1e-3, 1e-6);

  auto model = world->GetModel("box");
  ASSERT_NE(model, nullptr);

  auto joint = model->GetJoint("joint1");
  ASSERT_NE(joint, nullptr);

  // Take a few steps and confirm that the model remains in place
  world->Step(50);
  EXPECT_NEAR(joint->GetVelocity(0), 0.0, 1e-2);
  EXPECT_NEAR(joint->GetAngle(0).Ign().Radian(), 0.0, 1e-3);

  // Prepare harness publishers
  auto detachPub =
    this->node->Advertise<msgs::GzString>("~/box/harness/detach");
  auto velocityPub =
    this->node->Advertise<msgs::GzString>("~/box/harness/velocity");
  msgs::GzString msg;

  // Lower the harness
  // even with a large velocity, it will only fall via gravity
  const double downVel = -10;
  msg.set_data(std::to_string(downVel));
  velocityPub->Publish(msg);
  common::Time::MSleep(30);
  world->Step(3.0 / dt);
  EXPECT_NEAR(joint->GetVelocity(0), -0.113, 1e-3);
  EXPECT_NEAR(joint->GetAngle(0).Ign().Radian(), -0.3375, 1e-4);

  // Stop and verify that it holds position
  const double stopPosition = joint->GetAngle(0).Ign().Radian();
  msg.set_data(std::to_string(0));
  velocityPub->Publish(msg);
  common::Time::MSleep(30);
  world->Step(1.0 / dt);

  EXPECT_NEAR(joint->GetVelocity(0), 0.0, 1e-2);
  EXPECT_NEAR(joint->GetAngle(0).Ign().Radian(), stopPosition, 1e-2);

  // Raise the harness
  const double upVel = 0.2;
  msg.set_data(std::to_string(upVel));
  velocityPub->Publish(msg);
  common::Time::MSleep(30);
  world->Step(1.0 / dt);

  EXPECT_NEAR(joint->GetVelocity(0), upVel, 2e-2);
  EXPECT_NEAR(joint->GetAngle(0).Ign().Radian(), -0.145, 1e-3);
}

TEST_P(Harness, LowerStopRaise)
{
  const std::string physicsEngine = GetParam();
  if (physicsEngine == "simbody" || physicsEngine == "dart")
  {
    gzerr << "Skipping test for "
          << physicsEngine
          << " since it doesn't support dynamic creation/destruction of joints"
          << ", see issues #862, #903."
          << std::endl;
    return;
  }
  LowerStopRaise(physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Harness, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
