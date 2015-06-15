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
#include "gazebo/rendering/AxisVisual.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class AxisVisual_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(AxisVisual_TEST, LinkOriginTest)
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

  // create origin visual for the link using msg Load
  gazebo::rendering::AxisVisualPtr axisVisual(
      new gazebo::rendering::AxisVisual("_LINK_ORIGIN_VISUAL_",
      linkDefaultVis));
  axisVisual->Load(linkDefaultMsg);

  // Check that it was added to the scene (by Load)
  EXPECT_EQ(scene->GetVisual("_LINK_ORIGIN_VISUAL_"), axisVisual);

   /* public: void ShowAxisRotation(unsigned int _axis, bool _show);
      public: void ShowAxisShaft(unsigned int _axis, bool _show);
      public: void ShowAxisHead(unsigned int _axis, bool _show);
      public: void ScaleXAxis(const math::Vector3 &_scale);
      public: void ScaleYAxis(const math::Vector3 &_scale);
      public: void ScaleZAxis(const math::Vector3 &_scale);
      public: void SetAxisMaterial(unsigned int _axis,
                                   const std::string &_material);
      public: void SetAxisVisible(unsigned int _axis, bool _visible);
*/

  // Remove it from the scene (Fini is called)
  scene->RemoveVisual(axisVisual);

  // Check that it was removed
  EXPECT_TRUE(scene->GetVisual("_LINK_ORIGIN_VISUAL_") == NULL);

  // Reset pointer
  axisVisual.reset();
  EXPECT_TRUE(axisVisual == NULL);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
