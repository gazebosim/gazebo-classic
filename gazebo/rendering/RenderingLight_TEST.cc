/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class Light_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(Light_TEST, LightPoseTest)
{
  Load("worlds/empty.world");

  rendering::ScenePtr scene = rendering::get_scene("default");

  if (!scene)
    scene = rendering::create_scene("default", false);

  EXPECT_TRUE(scene != nullptr);

  // create a visual as light parent
  rendering::VisualPtr parentVis(
      new rendering::Visual("light_parent", scene, false));
  parentVis->Load();

  ignition::math::Pose3d testPose(1, 2, 3, 0, 1.57, 3.14);
  parentVis->SetWorldPose(testPose);

  // create a light and attach it to parent visual
  rendering::LightPtr light(new gazebo::rendering::Light(scene));
  msgs::Light msg;
  msg.set_name("light_parent::light");
  light->LoadFromMsg(msg);

  // verify pose
  EXPECT_EQ(testPose, light->WorldPose());
}

/////////////////////////////////////////////////
TEST_F(Light_TEST, LightVisualTest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != nullptr);

  // create a light
  gazebo::rendering::LightPtr light(new gazebo::rendering::Light(scene));
  light->Load();

  std::string lightName = light->Name();
  gazebo::rendering::VisualPtr lightVisual = scene->GetVisual(lightName);
  EXPECT_TRUE(lightVisual != nullptr);

  scene->RemoveLight(light);

  // reset the pointer so that the destructor gets called
  light.reset();

  lightVisual = scene->GetVisual(lightName);
  EXPECT_TRUE(lightVisual == nullptr);
}

/////////////////////////////////////////////////
TEST_F(Light_TEST, CastShadows)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  ASSERT_TRUE(scene != nullptr);

  // create a directional light and verify type and cast shadows.
  gazebo::rendering::LightPtr dirLight(new gazebo::rendering::Light(scene));
  msgs::Light msg;
  msg.set_type(gazebo::msgs::Light::DIRECTIONAL);
  msg.set_cast_shadows(true);
  dirLight->LoadFromMsg(msg);
  EXPECT_EQ(dirLight->LightType(), "directional");
  EXPECT_TRUE(dirLight->CastShadows());
  scene->RemoveLight(dirLight);
  dirLight.reset();

  // create a spot light and verify type and cast shadows.
  gazebo::rendering::LightPtr spotLight(new gazebo::rendering::Light(scene));
  msg.set_type(gazebo::msgs::Light::SPOT);
  msg.set_cast_shadows(true);
  spotLight->LoadFromMsg(msg);
  EXPECT_EQ(spotLight->LightType(), "spot");
  // issue #2083: spot light does not cast shadows
  EXPECT_FALSE(spotLight->CastShadows());
  scene->RemoveLight(spotLight);
  spotLight.reset();

  // create a point light and verify type and cast shadows.
  gazebo::rendering::LightPtr pointLight(new gazebo::rendering::Light(scene));
  msg.set_type(gazebo::msgs::Light::POINT);
  msg.set_cast_shadows(true);
  pointLight->LoadFromMsg(msg);
  EXPECT_EQ(pointLight->LightType(), "point");
  // issue #2083: point light does not cast shadows
  EXPECT_FALSE(pointLight->CastShadows());
  scene->RemoveLight(pointLight);
  pointLight.reset();
}

//////////////////////////////////////////////////
TEST_F(Light_TEST, SetVisible)
{
  this->Load("worlds/empty.world");

  auto scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != nullptr);

  // Create a light
  gazebo::rendering::LightPtr light = gazebo::rendering::LightPtr(
      new gazebo::rendering::Light(scene));
  ASSERT_TRUE(light != nullptr);

  light->Load();

  EXPECT_TRUE(light->Visible());
  light->SetVisible(false);
  EXPECT_FALSE(light->Visible());
  light->SetVisible(true);
  EXPECT_TRUE(light->Visible());

  // disable visualization - light should still be visible
  light->ShowVisual(false);
  EXPECT_TRUE(light->Visible());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
