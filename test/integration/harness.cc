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
  /// \brief Detach the box and expect it to fall.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void DetachPaused(const std::string &_physicsEngine);

  /// \brief Detach the box and expect it to fall, while sim is unpaused
  /// and physics updates are unthrottled (running as fast as possible).
  /// This tests thread safety.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void DetachUnpaused(const std::string &_physicsEngine);

  /// \brief Lower, stop, then raise harness.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void LowerStopRaise(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
void Harness::DetachPaused(const std::string &_physicsEngine)
{
  Load("worlds/harness.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world();
  ASSERT_NE(world , nullptr);

  const auto gravity = world->Gravity();
  EXPECT_EQ(gravity, ignition::math::Vector3d(0, 0, -9.8));

  auto model = world->GetModel("box");
  ASSERT_NE(model, nullptr);

  auto physics = world->GetPhysicsEngine();
  ASSERT_NE(physics, nullptr);
  const double dt = physics->GetMaxStepSize();
  EXPECT_NEAR(dt, 1e-3, 1e-6);

  {
    // Wait for joint to load
    for (int i = 0; i < 1000; ++i)
    {
      if (model->GetJoint("joint1"))
      {
        break;
      }
      common::Time::MSleep(10);
    }
    auto joint = model->GetJoint("joint1");
    ASSERT_NE(joint, nullptr);

    // Take a few steps and confirm that the model remains in place
    world->Step(50);
    EXPECT_NEAR(joint->GetVelocity(0), 0.0, 1e-2);
    EXPECT_NEAR(joint->GetAngle(0).Ign().Radian(), 0.0, 1e-3);
  }

  // Detach message harness via transport topic
  auto detachPub =
    this->node->Advertise<msgs::GzString>("~/box/harness/detach");
  msgs::GzString msg;
  msg.set_data("true");
  detachPub->Publish(msg);

  // Need to take world step before joint can be deleted
  world->Step(1);
  for (int i = 0; i < 1000; ++i)
  {
    if (!model->GetJoint("joint1"))
    {
      break;
    }
    common::Time::MSleep(10);
  }
  EXPECT_EQ(model->GetJoint("joint1"), nullptr);

  // Now step forward and expect it to fall
  const auto initialPose = model->GetWorldPose().Ign();
  const double fallTime = 0.15;
  // subtract one since we already took 1 step after publishing to detach
  world->Step(fallTime / dt - 1);

  const auto vel = model->GetWorldLinearVel().Ign();
  EXPECT_NEAR(vel.X(), 0, 2e-3);
  EXPECT_NEAR(vel.Y(), 0, 2e-3);
  EXPECT_NEAR(vel.Z(), fallTime * gravity.Z(), 2e-3);
  EXPECT_EQ(model->GetWorldPose().Ign().Pos(),
            initialPose.Pos() + 0.5*gravity * std::pow(fallTime, 2));

  // Send another detach command and take some more world steps
  // to confirm it doesn't crash
  detachPub->Publish(msg);
  world->Step(15);

  // Send a velocity command and take some more world steps
  // to confirm it doesn't crash
  auto velocityPub =
    this->node->Advertise<msgs::GzString>("~/box/harness/velocity");
  msg.set_data(std::to_string(0.0));
  velocityPub->Publish(msg);
  world->Step(15);
}

TEST_P(Harness, DetachPaused)
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
  DetachPaused(physicsEngine);
}

////////////////////////////////////////////////////////////////////////
void Harness::DetachUnpaused(const std::string &_physicsEngine)
{
  Load("worlds/harness.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world();
  ASSERT_NE(world , nullptr);
  world->SetPaused(false);
  EXPECT_FALSE(world->IsPaused());

  const auto gravity = world->Gravity();
  EXPECT_EQ(gravity, ignition::math::Vector3d(0, 0, -9.8));

  auto model = world->GetModel("box");
  ASSERT_NE(model, nullptr);

  auto physics = world->GetPhysicsEngine();
  ASSERT_NE(physics, nullptr);
  const double dt = physics->GetMaxStepSize();
  EXPECT_NEAR(dt, 1e-3, 1e-6);
  physics->SetRealTimeUpdateRate(0.0);
  EXPECT_DOUBLE_EQ(physics->GetRealTimeUpdateRate(), 0.0);

  {
    // Wait for joint to load
    for (int i = 0; i < 1000; ++i)
    {
      if (model->GetJoint("joint1"))
      {
        break;
      }
      common::Time::MSleep(10);
    }
    auto joint = model->GetJoint("joint1");
    ASSERT_NE(joint, nullptr);

    // Take a few steps and confirm that the model remains in place
    common::Time::MSleep(50);
    EXPECT_NEAR(joint->GetVelocity(0), 0.0, 1e-1);
    EXPECT_NEAR(joint->GetAngle(0).Ign().Radian(), 0.0, 1e-2);
  }

  // Detach message harness via transport topic
  auto detachPub =
    this->node->Advertise<msgs::GzString>("~/box/harness/detach");
  msgs::GzString msg;
  msg.set_data("true");
  detachPub->Publish(msg);

  // Need to take a world step before joint can be deleted
  common::Time::MSleep(10);
  for (int i = 0; i < 400; ++i)
  {
    if (!model->GetJoint("joint1"))
    {
      break;
    }
    common::Time::MSleep(10);
  };
  EXPECT_EQ(model->GetJoint("joint1"), nullptr);
}

TEST_P(Harness, DetachUnpaused)
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
  DetachUnpaused(physicsEngine);
}

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

  // Wait for joint to load
  for (int i = 0; i < 1000; ++i)
  {
    if (model->GetJoint("joint1"))
    {
      break;
    }
    common::Time::MSleep(10);
  }
  auto joint = model->GetJoint("joint1");
  ASSERT_NE(joint, nullptr);

  // Take a few steps and confirm that the model remains in place
  world->Step(50);
  EXPECT_NEAR(joint->GetVelocity(0), 0.0, 1e-2);
  EXPECT_NEAR(joint->GetAngle(0).Ign().Radian(), 0.0, 1e-3);

  // Prepare harness publisher
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
