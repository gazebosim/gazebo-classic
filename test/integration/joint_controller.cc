/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/JointController.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test_config.h"

using namespace gazebo;
class JointControllerTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(JointControllerTest, PositionControl)
{
  Load("worlds/simple_arm_test.world", true);
  gazebo::physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  gazebo::physics::ModelPtr model = world->ModelByName("simple_arm");
  gazebo::physics::JointControllerPtr jointController =
    model->GetJointController();

  world->Step(100);

  ignition::transport::Node node;
  const std::string topic = "/" + model->GetScopedName() + "/joint_cmd";
  ignition::transport::Node::Publisher jointPub =
      node.Advertise<ignition::msgs::JointCmd>(topic);

  ignition::msgs::JointCmd msg;
  msg.set_name("simple_arm::arm_shoulder_pan_joint");
  msg.mutable_position()->set_target(1.0);
  msg.mutable_position()->set_p_gain(10);
  msg.mutable_position()->set_i_gain(0.1);
  msg.mutable_position()->set_d_gain(4.5);
  jointPub.Publish(msg);

  world->Step(5000);

  auto angle = model->GetJoint("arm_shoulder_pan_joint")->Position(0);

  EXPECT_NEAR(angle, 1.0, 0.1);
}

/////////////////////////////////////////////////
TEST_F(JointControllerTest, VelocityControl)
{
  Load("worlds/simple_arm_test.world", true);
  gazebo::physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  gazebo::physics::ModelPtr model = world->ModelByName("simple_arm");
  gazebo::physics::JointControllerPtr jointController =
    model->GetJointController();

  world->Step(100);

  ignition::transport::Node node;
  const std::string topic = "/" + model->GetScopedName() + "/joint_cmd";
  ignition::transport::Node::Publisher jointPub =
      node.Advertise<ignition::msgs::JointCmd>(topic);

  ignition::msgs::JointCmd msg;
  msg.set_name("simple_arm::arm_shoulder_pan_joint");
  msg.mutable_velocity()->set_target(0.2);
  msg.mutable_velocity()->set_p_gain(10.0);
  msg.mutable_velocity()->set_i_gain(0.1);
  msg.mutable_velocity()->set_d_gain(0.1);
  jointPub.Publish(msg);

  world->Step(5000);
  double vel = model->GetJoint("arm_shoulder_pan_joint")->GetVelocity(0);

  EXPECT_NEAR(vel, 0.2, 0.05);
}

/////////////////////////////////////////////////
TEST_F(JointControllerTest, JointCmd)
{
  Load("worlds/simple_arm_test.world", true);
  gazebo::physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  gazebo::physics::ModelPtr model = world->ModelByName("simple_arm");
  gazebo::physics::JointControllerPtr jointController =
    model->GetJointController();

  // Pick one of the joints
  const std::string jointName =
      jointController->GetJoints().begin()->second->GetScopedName();

  // Check the default joint controller parameters
  std::map<std::string, double> forces = jointController->GetForces();
  std::map<std::string, double> positions = jointController->GetPositions();
  std::map<std::string, common::PID> posPids =
      jointController->GetPositionPIDs();
  std::map<std::string, double> velocities = jointController->GetVelocities();
  std::map<std::string, common::PID> velPids =
      jointController->GetVelocityPIDs();

  EXPECT_EQ(forces.size(), 0u);

  EXPECT_EQ(positions.size(), 0u);
  EXPECT_DOUBLE_EQ(posPids[jointName].GetPGain(), 1);
  EXPECT_DOUBLE_EQ(posPids[jointName].GetIGain(), 0.1);
  EXPECT_DOUBLE_EQ(posPids[jointName].GetDGain(), 0.01);

  EXPECT_EQ(velocities.size(), 0u);
  EXPECT_DOUBLE_EQ(velPids[jointName].GetPGain(), 1);
  EXPECT_DOUBLE_EQ(velPids[jointName].GetIGain(), 0.1);
  EXPECT_DOUBLE_EQ(velPids[jointName].GetDGain(), 0.01);

  // Set the joint controller parameters with a message
  ignition::transport::Node node;
  const std::string topic = "/" + model->GetScopedName() + "/joint_cmd";
  ignition::transport::Node::Publisher jointPub =
      node.Advertise<ignition::msgs::JointCmd>(topic);

  ignition::msgs::JointCmd msg;
  msg.set_name(jointName);
  msg.set_force(3);

  msg.mutable_position()->set_target(12.3);
  msg.mutable_position()->set_p_gain(4);
  msg.mutable_position()->set_i_gain(1);
  msg.mutable_position()->set_d_gain(9);

  msg.mutable_velocity()->set_target(3.21);
  msg.mutable_velocity()->set_p_gain(4);
  msg.mutable_velocity()->set_i_gain(1);
  msg.mutable_velocity()->set_d_gain(9);

  jointPub.Publish(msg);
  world->Step(5000);

  // Check the new joint controller parameters
  forces = jointController->GetForces();
  positions = jointController->GetPositions();
  posPids = jointController->GetPositionPIDs();
  velocities = jointController->GetVelocities();
  velPids = jointController->GetVelocityPIDs();

  EXPECT_EQ(forces.size(), 1u);
  EXPECT_DOUBLE_EQ(forces[jointName], 3);

  EXPECT_EQ(positions.size(), 1u);
  EXPECT_DOUBLE_EQ(positions[jointName], 12.3);
  EXPECT_DOUBLE_EQ(posPids[jointName].GetPGain(), 4);
  EXPECT_DOUBLE_EQ(posPids[jointName].GetIGain(), 1);
  EXPECT_DOUBLE_EQ(posPids[jointName].GetDGain(), 9);

  EXPECT_EQ(velocities.size(), 1u);
  EXPECT_DOUBLE_EQ(velocities[jointName], 3.21);
  EXPECT_DOUBLE_EQ(velPids[jointName].GetPGain(), 4);
  EXPECT_DOUBLE_EQ(velPids[jointName].GetIGain(), 1);
  EXPECT_DOUBLE_EQ(velPids[jointName].GetDGain(), 9);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
