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
#include "test/ServerFixture.hh"
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
  gazebo::physics::ModelPtr model = world->GetModel("simple_arm");
  gazebo::physics::JointControllerPtr jointController =
    model->GetJointController();

  world->Step(100);

  gazebo::transport::PublisherPtr pub =
    this->node->Advertise<gazebo::msgs::JointCmd>(
        "/gazebo/default/simple_arm/joint_cmd");

  msgs::JointCmd msg;
  msg.set_name("simple_arm::arm_shoulder_pan_joint");
  msg.mutable_position()->set_target(1.0);
  msg.mutable_position()->set_p_gain(10);
  msg.mutable_position()->set_i_gain(0.1);
  msg.mutable_position()->set_d_gain(4.5);
  pub->Publish(msg);

  world->Step(5000);

  math::Angle angle = model->GetJoint("arm_shoulder_pan_joint")->GetAngle(0);

  EXPECT_NEAR(angle.Radian(), 1.0, 0.1);
}

/////////////////////////////////////////////////
TEST_F(JointControllerTest, VelocityControl)
{
  Load("worlds/simple_arm_test.world", true);
  gazebo::physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  gazebo::physics::ModelPtr model = world->GetModel("simple_arm");
  gazebo::physics::JointControllerPtr jointController =
    model->GetJointController();

  world->Step(100);

  gazebo::transport::PublisherPtr pub =
    this->node->Advertise<gazebo::msgs::JointCmd>(
        "/gazebo/default/simple_arm/joint_cmd");

  msgs::JointCmd msg;
  msg.set_name("simple_arm::arm_shoulder_pan_joint");
  msg.mutable_velocity()->set_target(0.2);
  msg.mutable_velocity()->set_p_gain(10.0);
  msg.mutable_velocity()->set_i_gain(0.1);
  msg.mutable_velocity()->set_d_gain(0.1);
  pub->Publish(msg);

  world->Step(5000);
  double vel = model->GetJoint("arm_shoulder_pan_joint")->GetVelocity(0);

  EXPECT_NEAR(vel, 0.2, 0.05);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
