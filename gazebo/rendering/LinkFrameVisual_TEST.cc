/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in axispliance with the License.
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
#include "gazebo/rendering/LinkFrameVisual.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class LinkFrameVisual_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(LinkFrameVisual_TEST, LinkFrameTest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != NULL);

  // create a default link message
  gazebo::msgs::LinkPtr linkDefaultMsg;
  linkDefaultMsg.reset(new gazebo::msgs::Link);

  // create a link visual
  gazebo::rendering::VisualPtr linkDefaultVis;
  linkDefaultVis.reset(
      new gazebo::rendering::Visual("link", scene->GetWorldVisual()));

  // create frame visual for the link using msg Load
  gazebo::rendering::LinkFrameVisualPtr linkFrameVisual(
      new gazebo::rendering::LinkFrameVisual("_LINK_FRAME_VISUAL_",
      linkDefaultVis));
  linkFrameVisual->Load(linkDefaultMsg);

  // Check that it was added to the scene (by Load)
  EXPECT_EQ(scene->GetVisual("_LINK_FRAME_VISUAL_"), linkFrameVisual);

  // Check that it has type physics
  EXPECT_EQ(linkFrameVisual->GetType(), gazebo::rendering::Visual::VT_PHYSICS);

  // Check that the link visual is the parent
  EXPECT_EQ(linkFrameVisual->GetParent(), linkDefaultVis);

  // Check that the pose within the link visual (local pose) is zero
  EXPECT_EQ(linkFrameVisual->GetPose(), gazebo::math::Pose::Zero);

  // Remove it from the scene (Fini is called)
  scene->RemoveVisual(linkFrameVisual);

  // Check that it was removed
  EXPECT_TRUE(scene->GetVisual("_LINK_FRAME_VISUAL_") == NULL);

  // Reset pointer
  linkFrameVisual.reset();
  EXPECT_TRUE(linkFrameVisual == NULL);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
