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
  /// \brief Key to joint position
  /// \param[in] _physicsEngine Physics engine to use.
  public: void Position(const std::string &_physicsEngine);

  /// \brief Key to joint velocity
  /// \param[in] _physicsEngine Physics engine to use.
  public: void Velocity(const std::string &_physicsEngine);

  /// \brief Key to joint force
  /// \param[in] _physicsEngine Physics engine to use.
  public: void Force(const std::string &_physicsEngine);

  /// \brief One key to many joints
  /// \param[in] _physicsEngine Physics engine to use.
  public: void MultipleJoints(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
void KeysToJoints::Position(const std::string &_physicsEngine)
{
  this->Load("worlds/keys_to_joints.world", false, _physicsEngine);
  auto world = physics::get_world();
  ASSERT_NE(world , nullptr);

  auto model = world->ModelByName("position_teleop");
  ASSERT_NE(model, nullptr);

  // Setup keyboard publisher
  auto keyboardPub = this->node->Advertise<msgs::Any>("~/keyboard/keypress");

  // Position teleop
  auto joint = model->GetJoint("joint");
  ASSERT_NE(joint, nullptr);

  // Trigger key to increase position
  double target = joint->Position(0) + 0.05;

  msgs::Any msg;
  msg.set_type(msgs::Any_ValueType_INT32);
  msg.set_int_value(106);
  keyboardPub->Publish(msg);

  int maxSleep = 50;
  int sleep = 0;
  while (joint->Position(0) < target && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_GT(joint->Position(0), target);

  // Trigger key to decrease position
  target = joint->Position(0) - 0.05;
  msg.set_int_value(117);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (joint->Position(0) > target && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_LT(joint->Position(0), target);
}

TEST_P(KeysToJoints, Position)
{
  const std::string physicsEngine = GetParam();
  Position(physicsEngine);
}

////////////////////////////////////////////////////////////////////////
void KeysToJoints::Velocity(const std::string &_physicsEngine)
{
  this->Load("worlds/keys_to_joints.world", false, _physicsEngine);
  auto world = physics::get_world();
  ASSERT_NE(world , nullptr);

  auto model = world->ModelByName("velocity_teleop");
  ASSERT_NE(model, nullptr);

  // Setup keyboard publisher
  auto keyboardPub = this->node->Advertise<msgs::Any>("~/keyboard/keypress");

  // Velocity teleop
  auto joint = model->GetJoint("joint");
  ASSERT_NE(joint, nullptr);

  // Trigger key to have a positive velocity
  double target = 0.29;

  msgs::Any msg;
  msg.set_type(msgs::Any_ValueType_INT32);
  msg.set_int_value(107);
  keyboardPub->Publish(msg);

  int maxSleep = 50;
  int sleep = 0;
  while (joint->GetVelocity(0) < target && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_GT(joint->GetVelocity(0), target);

  // Trigger key to stop
  double tol = 1e-5;

  msg.set_int_value(105);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (std::abs(joint->GetVelocity(0)) > tol && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_LT(std::abs(joint->GetVelocity(0)), tol);

  // Trigger key to have a negative velocity
  msg.set_int_value(56);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (joint->GetVelocity(0) > -target && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_LT(joint->GetVelocity(0), -target);
}

TEST_P(KeysToJoints, Velocity)
{
  const std::string physicsEngine = GetParam();
  Velocity(physicsEngine);
}

////////////////////////////////////////////////////////////////////////
void KeysToJoints::Force(const std::string &_physicsEngine)
{
  this->Load("worlds/keys_to_joints.world", false, _physicsEngine);
  auto world = physics::get_world();
  ASSERT_NE(world , nullptr);

  auto model = world->ModelByName("force_teleop");
  ASSERT_NE(model, nullptr);

  // Setup keyboard publisher
  auto keyboardPub = this->node->Advertise<msgs::Any>("~/keyboard/keypress");

  // Force teleop
  auto joint = model->GetJoint("joint");
  ASSERT_NE(joint, nullptr);

  // Trigger key to push in positive direction
  double target = joint->Position(0) + 0.1;

  msgs::Any msg;
  msg.set_type(msgs::Any_ValueType_INT32);
  msg.set_int_value(108);
  keyboardPub->Publish(msg);

  // DART only responds after a few clicks: issue #2091
  if (_physicsEngine == "dart")
  {
    for (int i = 0; i < 5; ++i)
      keyboardPub->Publish(msg);
  }

  int maxSleep = 50;
  int sleep = 0;
  while (joint->Position(0) < target && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_GT(joint->Position(0), target);

  // Trigger key to stop
  if (_physicsEngine == "dart")
  {
    gzerr << "Skipping rest of test for [dart] due to issue #2091" << std::endl;
    return;
  }

  double tol = 0.1;

  msg.set_int_value(111);
  keyboardPub->Publish(msg);

  sleep = 0;
  while (std::abs(joint->GetVelocity(0)) > tol && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_LT(std::abs(joint->GetVelocity(0)), tol);
}

TEST_P(KeysToJoints, Force)
{
  const std::string physicsEngine = GetParam();
  if (physicsEngine == "simbody")
  {
    gzerr << "Skipping test for ["
          << physicsEngine
          << "] due to Joint::SetForce, see issue #2092"
          << std::endl;
    return;
  }
  if (physicsEngine == "bullet")
  {
    gzerr << "Skipping test for ["
          << physicsEngine
          << "] because of thread safety, see issue #2098"
          << std::endl;
    return;
  }
  Force(physicsEngine);
}

////////////////////////////////////////////////////////////////////////
void KeysToJoints::MultipleJoints(const std::string &_physicsEngine)
{
  this->Load("worlds/keys_to_joints.world", false, _physicsEngine);
  auto world = physics::get_world();
  ASSERT_NE(world , nullptr);

  auto model = world->ModelByName("multiple_teleop");
  ASSERT_NE(model, nullptr);

  // Setup keyboard publisher
  auto keyboardPub = this->node->Advertise<msgs::Any>("~/keyboard/keypress");

  // MultipleJoints teleop
  auto joint2 = model->GetJoint("joint_2");
  ASSERT_NE(joint2, nullptr);

  auto joint3 = model->GetJoint("joint_3");
  ASSERT_NE(joint3, nullptr);

  // Trigger a single key which increases joint2 and decreases joint3
  double target2 = joint2->Position(0) + 0.05;
  double target3 = joint3->Position(0) - 0.05;

  msgs::Any msg;
  msg.set_type(msgs::Any_ValueType_INT32);
  msg.set_int_value(59);
  keyboardPub->Publish(msg);

  int maxSleep = 50;
  int sleep = 0;
  while (joint2->Position(0) < target2 &&
         joint3->Position(0) > target3 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_GT(joint2->Position(0), target2);
  EXPECT_LT(joint3->Position(0), target3);
}

TEST_P(KeysToJoints, MultipleJoints)
{
  const std::string physicsEngine = GetParam();
  MultipleJoints(physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, KeysToJoints, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
