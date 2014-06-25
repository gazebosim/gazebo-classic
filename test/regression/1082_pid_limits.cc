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
class Issue1082Test : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(Issue1082Test, PIDLimitsVelocity)
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

  math::Pose startPose = model->GetLink("arm_elbow_pan")->GetWorldPose();

  msgs::JointCmd msg;
  msg.set_name("simple_arm::arm_shoulder_pan_joint");
  msg.mutable_velocity()->set_target(10);
  msg.mutable_velocity()->set_p_gain(1);
  msg.mutable_velocity()->set_i_gain(.1);
  msg.mutable_velocity()->set_d_gain(.01);
  msg.mutable_velocity()->set_i_max(0.5);
  msg.mutable_velocity()->set_i_min(0.1);
  msg.mutable_velocity()->set_limit(0.001);
  pub->Publish(msg);

  world->Step(500);

  math::Pose endPose = model->GetLink("arm_elbow_pan")->GetWorldPose();

  double diffDist = (startPose - endPose).pos.GetLength();

  gzdbg << "Start[" << startPose << "] End[" << endPose << "] Dist["
        << diffDist << "]\n";

  // 0.002 chosen arbitrarily.
  EXPECT_LT(diffDist, 0.002);
  EXPECT_GT(diffDist, -0.002);
}

/////////////////////////////////////////////////
TEST_F(Issue1082Test, PIDLimitsPosition)
{
  // Load an empty world
  Load("worlds/simple_arm_test.world", true);
  gazebo::physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  gazebo::physics::ModelPtr model = world->GetModel("simple_arm");
  gazebo::physics::JointControllerPtr jointController =
    model->GetJointController();

  gazebo::transport::PublisherPtr pub =
    this->node->Advertise<gazebo::msgs::JointCmd>(
        "/gazebo/default/simple_arm/joint_cmd");

  math::Pose startPose = model->GetLink("arm_elbow_pan")->GetWorldPose();

  msgs::JointCmd msg;
  msg.set_name("simple_arm::arm_shoulder_pan_joint");
  msg.mutable_position()->set_target(10);
  msg.mutable_position()->set_p_gain(1);
  msg.mutable_position()->set_i_gain(.1);
  msg.mutable_position()->set_d_gain(.01);
  msg.mutable_position()->set_i_max(0.5);
  msg.mutable_position()->set_i_min(0.1);
  msg.mutable_position()->set_limit(0.001);
  pub->Publish(msg);

  world->Step(500);

  math::Pose endPose = model->GetLink("arm_elbow_pan")->GetWorldPose();

  gzdbg << "Start Pose[" << startPose << "]\n";
  gzdbg << "End Pose[" << endPose << "]\n";

  double diffDist = (startPose - endPose).pos.GetLength();

  EXPECT_LT(diffDist, 0.002);
  EXPECT_GT(diffDist, -0.002);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
