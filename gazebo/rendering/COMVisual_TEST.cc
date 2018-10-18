/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/COMVisual.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class COMVisual_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(COMVisual_TEST, COMVisualTest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != nullptr);

  // get scene visual child count before we create any visuals
  EXPECT_TRUE(scene->WorldVisual() != nullptr);
  unsigned int count = scene->WorldVisual()->GetChildCount();

  // create a default link message
  gazebo::msgs::LinkPtr linkDefaultMsg;
  linkDefaultMsg.reset(new gazebo::msgs::Link);

  // create a link visual
  gazebo::rendering::VisualPtr linkDefaultVis;
  linkDefaultVis.reset(
      new gazebo::rendering::Visual("link", scene->WorldVisual()));

  // create CoMVisual for the link using msg Load
  gazebo::rendering::COMVisualPtr comDefaultVis(
      new gazebo::rendering::COMVisual("_COM_VISUAL_", linkDefaultVis));
  comDefaultVis->Load(linkDefaultMsg);

  EXPECT_EQ(comDefaultVis->GetInertiaPose().pos, math::Vector3::Zero);
  EXPECT_EQ(comDefaultVis->GetInertiaPose().rot.GetAsEuler(),
      math::Vector3::Zero);

  // Create a message and set inertia pose
  math::Vector3 pos(1, 0, -3);
  math::Quaternion quat(M_PI/2, 0, -M_PI/5);

  gazebo::msgs::Link linkMsg;
  linkMsg.set_name("link");
  msgs::Set(linkMsg.mutable_inertial()->mutable_pose()->mutable_position(),
      pos.Ign());
  msgs::Set(linkMsg.mutable_inertial()->mutable_pose()->mutable_orientation(),
      quat.Ign());

  // create a link visual
  gazebo::rendering::VisualPtr linkVis;
  linkVis.reset(
      new gazebo::rendering::Visual("link", scene->WorldVisual()));

  // create CoMVisual for the link using SDF Load
  gazebo::rendering::COMVisualPtr comVis(
      new gazebo::rendering::COMVisual("_COM_VISUAL_", linkVis));
  comVis->Load(msgs::LinkToSDF(linkMsg));

  EXPECT_EQ(comVis->GetInertiaPose().pos, pos);
  EXPECT_EQ(comVis->GetInertiaPose().rot, quat);

  // test destroying the visual
  comVis->Fini();
  EXPECT_EQ(comVis->GetChildCount(), 0u);

  // verify scene's child count is the same as before the visual was created
  EXPECT_EQ(scene->WorldVisual()->GetChildCount(), count);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
