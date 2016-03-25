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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Gripper.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Joint.hh"

using namespace gazebo;
class GripperTest : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test to make sure the gripper forms a joint when grasping an object
TEST_F(GripperTest, Close)
{
gzdbg << "A" << std::endl;
  this->Load("worlds/gripper.world");
gzdbg << "A" << std::endl;
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->GetModel("simple_gripper");
  ASSERT_TRUE(model != NULL);

  physics::JointPtr leftJoint = model->GetJoint("palm_left_finger");
  ASSERT_TRUE(leftJoint != NULL);

  physics::JointPtr rightJoint = model->GetJoint("palm_right_finger");
  ASSERT_TRUE(rightJoint != NULL);

gzdbg << "A" << std::endl;
  physics::GripperPtr gripper = model->GetGripper(0);
gzdbg << "A" << std::endl;
  ASSERT_TRUE(gripper != NULL);
gzdbg << "A5" << std::endl;

  // The gripper should not be attached to anything
  EXPECT_FALSE(gripper->IsAttached());
gzdbg << "A" << std::endl;

  // Close the gripper.
  leftJoint->SetForce(0, -0.5);
  rightJoint->SetForce(0, 0.5);

  auto nodePtr = transport::NodePtr(new transport::Node());
  nodePtr->Init();
  transport::PublisherPtr jointPub = nodePtr->Advertise<msgs::JointCmd>(
        "~/simple_gripper/joint_cmd");
gzdbg << "A" << std::endl;

  msgs::JointCmd msg;
  msg.set_name("simple_gripper::palm_right_finger");
  msg.set_force(0.6);
  jointPub->Publish(msg);
gzdbg << "A" << std::endl;

  msg.set_name("simple_gripper::palm_left_finger");
  msg.set_force(-0.6);
  jointPub->Publish(msg);
gzdbg << "A10" << std::endl;

  int i = 0;
  while (!gripper->IsAttached() && i < 100)
  {
gzdbg << "loop" << std::endl;
    common::Time::MSleep(500);
    ++i;
  }
  EXPECT_LT(i, 100);
gzdbg << "A" << std::endl;

  // The gripper should be attached.
  EXPECT_TRUE(gripper->IsAttached());
gzdbg << "end" << std::endl;
}

/////////////////////////////////////////////////
// \brief Test grasp can close and open
TEST_F(GripperTest, CloseOpen)
{
gzdbg << "A" << std::endl;
  Load("worlds/gripper.world");
gzdbg << "A" << std::endl;
  physics::WorldPtr world = physics::get_world("default");
gzdbg << "A" << std::endl;
  ASSERT_TRUE(world != NULL);
gzdbg << "A" << std::endl;

  physics::ModelPtr model = world->GetModel("simple_gripper");
gzdbg << "A5" << std::endl;
  ASSERT_TRUE(model != NULL);
gzdbg << "A" << std::endl;

  physics::JointPtr leftJoint = model->GetJoint("palm_left_finger");
gzdbg << "A" << std::endl;
  ASSERT_TRUE(leftJoint != NULL);
gzdbg << "A" << std::endl;

  physics::JointPtr rightJoint = model->GetJoint("palm_right_finger");
gzdbg << "A" << std::endl;
  ASSERT_TRUE(rightJoint != NULL);
gzdbg << "A10" << std::endl;

  physics::GripperPtr gripper = model->GetGripper(0);
gzdbg << "A" << std::endl;
  ASSERT_TRUE(gripper != NULL);
gzdbg << "A" << std::endl;

  // The gripper should not be attached to anything
  EXPECT_FALSE(gripper->IsAttached());
gzdbg << "A" << std::endl;

  // Close the gripper.
  leftJoint->SetForce(0, -0.5);
  rightJoint->SetForce(0, 0.5);

  auto nodePtr = transport::NodePtr(new transport::Node());
  nodePtr->Init();
  transport::PublisherPtr jointPub = nodePtr->Advertise<msgs::JointCmd>(
        "~/simple_gripper/joint_cmd");
gzdbg << "A" << std::endl;

  msgs::JointCmd msg;
  msg.set_name("simple_gripper::palm_right_finger");
  msg.set_force(0.6);
  jointPub->Publish(msg);
gzdbg << "A" << std::endl;

  msg.set_name("simple_gripper::palm_left_finger");
  msg.set_force(-0.6);
  jointPub->Publish(msg);
gzdbg << "A" << std::endl;

  int i = 0;
  while (!gripper->IsAttached() && i < 100)
  {
gzdbg << "loop" << std::endl;
    common::Time::MSleep(500);
    ++i;
  }
  EXPECT_LT(i, 100);
gzdbg << "A" << std::endl;

  // The gripper should be attached.
  EXPECT_TRUE(gripper->IsAttached());
gzdbg << "A" << std::endl;

  // Open the gripper.
  msg.set_name("simple_gripper::palm_right_finger");
  msg.set_force(-0.6);
  jointPub->Publish(msg);
gzdbg << "A" << std::endl;

  msg.set_name("simple_gripper::palm_left_finger");
  msg.set_force(0.6);
  jointPub->Publish(msg);
gzdbg << "A" << std::endl;

  i = 0;
  while (gripper->IsAttached() && i < 100)
  {
gzdbg << "loop" << std::endl;
    common::Time::MSleep(500);
    ++i;
  }
  EXPECT_LT(i, 100);
gzdbg << "A" << std::endl;

  // The gripper should release the box.
  EXPECT_FALSE(gripper->IsAttached());
gzdbg << "end" << std::endl;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
