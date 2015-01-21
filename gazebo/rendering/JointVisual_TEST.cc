/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/math/Rand.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/JointVisual.hh"
#include "test/ServerFixture.hh"

class JointVisual_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(JointVisual_TEST, JointVisualTest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != NULL);

  // create a fake child visual where the joint visual will be attached to
  gazebo::rendering::VisualPtr childVis;
  childVis.reset(
      new gazebo::rendering::Visual("child", scene->GetWorldVisual()));

  // create a joint message for testing
  gazebo::msgs::JointPtr jointMsg;
  jointMsg.reset(new gazebo::msgs::Joint);
  jointMsg->set_parent(scene->GetWorldVisual()->GetName());
  jointMsg->set_parent_id(scene->GetWorldVisual()->GetId());
  jointMsg->set_child(childVis->GetName());
  jointMsg->set_child_id(childVis->GetId());
  jointMsg->set_name("test_joint");
  jointMsg->set_id(11111);
  msgs::Set(jointMsg->mutable_pose(), math::Pose(1, 2, 3, 1.57, 1.57, 0));
  jointMsg->set_type(msgs::Joint::REVOLUTE2);
  jointMsg->add_angle(1.2);
  {
    msgs::Axis *axis1 = jointMsg->mutable_axis1();
    msgs::Set(axis1->mutable_xyz(), gazebo::math::Vector3(0, 1, 0));
    axis1->set_limit_lower(-1.2);
    axis1->set_limit_upper(2.3);
    axis1->set_limit_effort(6);
    axis1->set_limit_velocity(1);
    axis1->set_damping(true);
    axis1->set_friction(true);
    axis1->set_use_parent_model_frame(true);
  }
  jointMsg->add_angle(-1.2);
  {
    msgs::Axis *axis2 = jointMsg->mutable_axis2();
    msgs::Set(axis2->mutable_xyz(), gazebo::math::Vector3(0, 0, 1));
    axis2->set_limit_lower(-1.2);
    axis2->set_limit_upper(-0.3);
    axis2->set_limit_effort(3);
    axis2->set_limit_velocity(2);
    axis2->set_damping(false);
    axis2->set_friction(false);
    axis2->set_use_parent_model_frame(false);
  }

  // test calling constructor and Load functions and make sure
  // there are no segfaults
  gazebo::rendering::JointVisualPtr jointVis(
      new gazebo::rendering::JointVisual(
      "model_GUIONLY_joint_vis", childVis));
  jointVis->Load(jointMsg);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
