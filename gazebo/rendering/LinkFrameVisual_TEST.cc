/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
class LinkFrameVisual_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(LinkFrameVisual_TEST, LinkFrameTest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != nullptr);

  // get scene visual child count before we create any visuals
  EXPECT_TRUE(scene->WorldVisual() != nullptr);
  unsigned int count = scene->WorldVisual()->GetChildCount();

  // create a link visual
  gazebo::rendering::VisualPtr linkVis;
  linkVis.reset(
      new gazebo::rendering::Visual("link", scene->WorldVisual()));

  // create frame visual for the link
  gazebo::rendering::LinkFrameVisualPtr linkFrameVis(
      new gazebo::rendering::LinkFrameVisual("_LINK_FRAME_VISUAL_",
      linkVis));
  linkFrameVis->Load();

  // Check that it was added to the scene (by Load)
  EXPECT_EQ(scene->GetVisual("_LINK_FRAME_VISUAL_"), linkFrameVis);

  // Check that it has type physics
  EXPECT_EQ(linkFrameVis->GetType(), gazebo::rendering::Visual::VT_PHYSICS);

  // Check that the link visual is the parent
  EXPECT_EQ(linkFrameVis->GetParent(), linkVis);

  // Check that the pose within the link visual (local pose) is zero
  EXPECT_EQ(linkFrameVis->GetPose(), gazebo::math::Pose::Zero);

  // Check that frame is not highlighted
  EXPECT_FALSE(linkFrameVis->GetHighlighted());

  // Set  highlighted
  linkFrameVis->SetHighlighted(true);

  // Check that frame is highlighted
  EXPECT_TRUE(linkFrameVis->GetHighlighted());

  // Remove it from the scene (Fini is called)
  scene->RemoveVisual(linkFrameVis);

  // Check that it was removed
  EXPECT_TRUE(scene->GetVisual("_LINK_FRAME_VISUAL_") == nullptr);

  // Reset pointer
  linkFrameVis.reset();
  EXPECT_TRUE(linkFrameVis == nullptr);

  // verify scene's child count is the same as before the visual was created
  EXPECT_EQ(scene->WorldVisual()->GetChildCount(), count);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
