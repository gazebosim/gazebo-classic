/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/test/ServerFixture.hh"


using namespace gazebo;
class RTShaderSystem_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(RTShaderSystem_TEST, Shadows)
{
  Load("worlds/shapes.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != nullptr);

  scene->SetShadowsEnabled(true);
  EXPECT_TRUE(scene->ShadowsEnabled());

  rendering::RTShaderSystem *shaderSys =
      rendering::RTShaderSystem::Instance();

  shaderSys->SetShadowTextureSize(128u);
  EXPECT_EQ(128u, shaderSys->ShadowTextureSize());

  shaderSys->SetShadowClipDist(0.3, 105);
  EXPECT_DOUBLE_EQ(0.3, shaderSys->ShadowNearClip());
  EXPECT_DOUBLE_EQ(105, shaderSys->ShadowFarClip());

  shaderSys->SetShadowSplitLambda(1.2);
  EXPECT_DOUBLE_EQ(1.2, shaderSys->ShadowSplitLambda());

  shaderSys->SetShadowSplitPadding(4.8);
  EXPECT_DOUBLE_EQ(4.8, shaderSys->ShadowSplitPadding());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
