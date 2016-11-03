/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;
class KeysToJoints : public ServerFixture,
                public testing::WithParamInterface<const char*>
{
  /// \brief Key to joint commands
  /// \param[in] _physicsEngine Physics engine to use.
  public: void Commands(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
void KeysToJoints::Commands(const std::string &_physicsEngine)
{
  this->Load("worlds/simple_arm_teleop.world", false, _physicsEngine);
  auto world = physics::get_world();
  ASSERT_NE(world , nullptr);

  auto model = world->GetModel("simple_arm");
  ASSERT_NE(model, nullptr);

  // Setup keyboard publisher
  auto keyboardPub = this->node->Advertise<msgs::Any>("~/keyboard/keypress");

  // Position teleop
  auto joint = model->GetJoint("simple_arm::arm_wrist_lift_joint");
  ASSERT_NE(joint, nullptr);

  // Wait to reach initial value
  double tol = 1e-5;
  if (_physicsEngine == "dart")
    tol = 0.001;

  double target = 0.05;
  double current = -0.8;

  int maxSleep = 30;
  int sleep = 0;
  while (std::abs(current - joint->GetAngle(0).Radian()) > tol && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_LT(std::abs(current - joint->GetAngle(0).Radian()), tol);

  // Trigger key to increase position
  current = joint->GetAngle(0).Radian();
  msgs::Any msg;
  msg.set_type(msgs::Any_ValueType_INT32);
  msg.set_int_value(107);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (std::abs(current - joint->GetAngle(0).Radian()) < target &&
      sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_GT(std::abs(current - joint->GetAngle(0).Radian()), target);
  EXPECT_LT(current, joint->GetAngle(0).Radian());

  // Trigger key to decrease position
  current = joint->GetAngle(0).Radian();
  msg.set_int_value(105);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (std::abs(current - joint->GetAngle(0).Radian()) < target &&
      sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_GT(std::abs(current - joint->GetAngle(0).Radian()), target);
  EXPECT_GT(current, joint->GetAngle(0).Radian());

  // Velocity teleop
  joint = model->GetJoint("simple_arm::arm_shoulder_pan_joint");
  ASSERT_NE(joint, nullptr);

  // Wait to reach initial value
  current = 0;
  target = 0.09;

  tol = 1e-5;
  // TODO: Figure out why ode never stops
  if (_physicsEngine == "ode")
    tol = 0.2;
  else if (_physicsEngine == "dart")
    tol = 1e-4;

  sleep = 0;
  while (std::abs(current - joint->GetVelocity(0)) > tol && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_LT(std::abs(current - joint->GetVelocity(0)), tol);

  // Trigger key to have a positive velocity
  msg.set_int_value(104);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (joint->GetVelocity(0) < target && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_GT(joint->GetVelocity(0), target);

  // Trigger key to have a negative velocity
  msg.set_int_value(54);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (joint->GetVelocity(0) > -target && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_LT(joint->GetVelocity(0), -target);

  // Trigger key to stop
  msg.set_int_value(121);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (std::abs(joint->GetVelocity(0)) > tol && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_LT(std::abs(joint->GetVelocity(0)), tol);

  // Force teleop
  joint = model->GetJoint("simple_arm::arm_elbow_pan_joint");
  ASSERT_NE(joint, nullptr);

  target = 0.015;

  // Trigger key to push in positive direction
  current = joint->GetAngle(0).Radian();
  msg.set_int_value(106);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (std::abs(current - joint->GetAngle(0).Radian()) < target &&
      sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_GT(std::abs(current - joint->GetAngle(0).Radian()), target);
  EXPECT_LT(current, joint->GetAngle(0).Radian());

  // Trigger key to decrease position
  current = joint->GetAngle(0).Radian();
  msg.set_int_value(117);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (std::abs(current - joint->GetAngle(0).Radian()) < target &&
      sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_GT(std::abs(current - joint->GetAngle(0).Radian()), target);
  EXPECT_GT(current, joint->GetAngle(0).Radian());
}

TEST_P(KeysToJoints, Commands)
{
  const std::string physicsEngine = GetParam();
  if (physicsEngine == "simbody")
  {
    gzerr << "Skipping test for ["
          << physicsEngine
          << "] since it doesn't load the simple arm"
          << ", see gazebo_models issue #33."
          << std::endl;
    return;
  }
  Commands(physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, KeysToJoints, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
