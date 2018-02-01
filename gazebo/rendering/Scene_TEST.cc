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
#include "gazebo/rendering/Scene.hh"
#include "gazebo/test/ServerFixture.hh"


using namespace gazebo;
class Scene_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(Scene_TEST, AddRemoveCameras)
{
  Load("worlds/empty.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != nullptr);

  // verify no cameras are currently in the scene
  EXPECT_EQ(scene->CameraCount(), 0u);

  // create a camera and verify count
  rendering::CameraPtr camera = scene->CreateCamera("test_camera", false);
  EXPECT_EQ(scene->CameraCount(), 1u);
  EXPECT_TRUE(scene->GetCamera("test_camera") == camera);

  // create another camera and verify count
  rendering::CameraPtr camera2 = scene->CreateCamera("test_camera2", false);
  EXPECT_EQ(scene->CameraCount(), 2u);
  EXPECT_TRUE(scene->GetCamera("test_camera2") == camera2);

  // Remove a camera and check that it has been removed
  scene->RemoveCamera(camera->Name());
  EXPECT_EQ(scene->CameraCount(), 1u);
  EXPECT_TRUE(scene->GetCamera("test_camera") == nullptr);
  EXPECT_TRUE(scene->GetCamera("test_camera2") != nullptr);

  // remove non-existent camera
  scene->RemoveCamera("no_such_camera");
  EXPECT_EQ(scene->CameraCount(), 1u);
  EXPECT_TRUE(scene->GetCamera("test_camera") == nullptr);
  EXPECT_TRUE(scene->GetCamera("test_camera2") != nullptr);

  // Remove the remaining camera and check that it has been removed
  scene->RemoveCamera(camera2->Name());
  EXPECT_EQ(scene->CameraCount(), 0u);
  EXPECT_TRUE(scene->GetCamera("test_camera") == nullptr);
  EXPECT_TRUE(scene->GetCamera("test_camera2") == nullptr);
}

/////////////////////////////////////////////////
TEST_F(Scene_TEST, AddRemoveVisuals)
{
  Load("worlds/empty.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != nullptr);

  // Check that it has two visuals, the world and origin visuals
  EXPECT_EQ(scene->VisualCount(), 2u);
  EXPECT_TRUE(scene->GetVisual("__world_node__") != nullptr);

  // Add a visual and check that it has been added
  rendering::VisualPtr visual1;
  visual1.reset(new rendering::Visual("visual1", scene));
  scene->AddVisual(visual1);
  EXPECT_EQ(scene->VisualCount(), 3u);
  EXPECT_TRUE(scene->GetVisual("visual1") != nullptr);

  // Add a visual and check that it has been added
  rendering::VisualPtr visual2;
  visual2.reset(new rendering::Visual("visual2", scene));
  scene->AddVisual(visual2);
  EXPECT_EQ(scene->VisualCount(), 4u);
  EXPECT_TRUE(scene->GetVisual("visual2") != nullptr);

  // Remove a visual and check that it has been removed
  scene->RemoveVisual(visual1);
  EXPECT_EQ(scene->VisualCount(), 3u);
  EXPECT_FALSE(scene->GetVisual("visual1"));
}

/////////////////////////////////////////////////
TEST_F(Scene_TEST, RemoveModelVisual)
{
  // Load a world containing 3 simple shapes
  Load("worlds/shapes.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != nullptr);

  // Wait until all models are inserted
  int sleep = 0;
  int maxSleep = 10;
  rendering::VisualPtr box, sphere, cylinder;
  while ((!box || !sphere || !cylinder) && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    box = scene->GetVisual("box");
    cylinder = scene->GetVisual("cylinder");
    sphere = scene->GetVisual("sphere");
    common::Time::MSleep(1000);
    sleep++;
  }

  // Check that the model visuals were properly added
  ASSERT_TRUE(box != nullptr);
  ASSERT_TRUE(sphere != nullptr);
  ASSERT_TRUE(cylinder != nullptr);

  const std::vector<std::string> models = {
    "box",
    "cylinder",
    "sphere"};

  // Turn on visualizations
  for (auto const &modelName : models)
  {
    scene->GetVisual(modelName)->ShowInertia(true);
    scene->GetVisual(modelName)->ShowCOM(true);
    scene->GetVisual(modelName)->ShowLinkFrame(true);
    scene->GetVisual(modelName)->ShowCollision(true);
  }

  const std::vector<std::string> suffixes = {
    "",
    "::link",
    "::link::visual",
    "::link::collision__COLLISION_VISUAL__",
    "::link_INERTIA_VISUAL__",
    "::link_COM_VISUAL__",
    "::link_LINK_FRAME_VISUAL__",
    "::link_LINK_FRAME_VISUAL___X_AXIS",
    "::link_LINK_FRAME_VISUAL___X_AXIS__SHAFT__",
    "::link_LINK_FRAME_VISUAL___X_AXIS__HEAD__",
    "::link_LINK_FRAME_VISUAL___X_AXIS__ROTATION__",
    "::link_LINK_FRAME_VISUAL___Y_AXIS",
    "::link_LINK_FRAME_VISUAL___Y_AXIS__SHAFT__",
    "::link_LINK_FRAME_VISUAL___Y_AXIS__HEAD__",
    "::link_LINK_FRAME_VISUAL___Y_AXIS__ROTATION__",
    "::link_LINK_FRAME_VISUAL___Z_AXIS",
    "::link_LINK_FRAME_VISUAL___Z_AXIS__SHAFT__",
    "::link_LINK_FRAME_VISUAL___Z_AXIS__HEAD__",
    "::link_LINK_FRAME_VISUAL___Z_AXIS__ROTATION__"};

  for (auto const &modelName : models)
  {
    for (auto const &suffix : suffixes)
    {
      EXPECT_TRUE(scene->GetVisual(modelName + suffix) != nullptr)
          << "check visual exists: " << modelName + suffix;
    }
  }

  // Send request to delete the box model
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::requestNoReply(node, "entity_delete", "box");

  sleep = 0;
  while (box && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    box = scene->GetVisual("box");
    common::Time::MSleep(1000);
    sleep++;
  }
  ASSERT_TRUE(box == nullptr);

  for (auto const &suffix : suffixes)
  {
    EXPECT_TRUE(scene->GetVisual("box" + suffix) == nullptr)
        << "check visual does not exist: " << "box" + suffix;
  }
}

/////////////////////////////////////////////////
TEST_F(Scene_TEST, VisualType)
{
  // Load a world containing 3 simple shapes
  Load("worlds/shapes.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != nullptr);

  // Spawn another box. The model loading process is slightly different so
  // check that its visuals also have the correct visual type.
  SpawnBox("new_box",
      ignition::math::Vector3d(1, 1, 1),
      ignition::math::Vector3d(10, 10, 1),
      ignition::math::Vector3d::Zero);

  // Wait until all models are inserted
  int sleep = 0;
  int maxSleep = 10;
  rendering::VisualPtr box, sphere, cylinder, newBox;
  while ((!box || !sphere || !cylinder || !newBox) && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    box = scene->GetVisual("box");
    cylinder = scene->GetVisual("cylinder");
    sphere = scene->GetVisual("sphere");
    newBox = scene->GetVisual("new_box");
    common::Time::MSleep(1000);
    sleep++;
  }

  // Check that the model visuals were properly added
  ASSERT_TRUE(box != nullptr);
  ASSERT_TRUE(sphere != nullptr);
  ASSERT_TRUE(cylinder != nullptr);
  ASSERT_TRUE(newBox != nullptr);

  const std::vector<std::string> models = {
    "box",
    "cylinder",
    "sphere",
    "new_box",
  };

  for (auto const &modelName : models)
  {
    gzmsg << "Checking model [" << modelName << "]" << std::endl;

    auto vis = scene->GetVisual(modelName);
    ASSERT_TRUE(vis != nullptr);

    // Turn on visualizations
    vis->ShowCollision(true);
    vis->ShowInertia(true);
    vis->ShowLinkFrame(true);
    vis->ShowCOM(true);

    // Verify type is VT_MODEL
    EXPECT_TRUE(vis->GetType() == rendering::Visual::VT_MODEL);

    // Check that the link visuals were properly added
    rendering::VisualPtr v;
    if (modelName != "new_box")
      v = scene->GetVisual(modelName + "::link");
    else
      v = scene->GetVisual(modelName + "::body");

    ASSERT_TRUE(v != nullptr);

    // Verify type is VT_LINK
    EXPECT_TRUE(v->GetType() == rendering::Visual::VT_LINK);

    // Check that the visual visuals were properly added
    if (modelName != "new_box")
      v = scene->GetVisual(modelName + "::link::visual");
    else
      v = scene->GetVisual(modelName + "::body::visual");

    ASSERT_TRUE(v != nullptr);

    // Verify type is VT_VISUAL
    EXPECT_TRUE(v->GetType() == rendering::Visual::VT_VISUAL);

    // Check that the collision visuals were properly added
    if (modelName != "new_box")
      v = scene->GetVisual(modelName + "::link::collision__COLLISION_VISUAL__");
    else
      v = scene->GetVisual(modelName + "::body::geom__COLLISION_VISUAL__");

    ASSERT_TRUE(v != nullptr);

    // Verify type is VT_COLLISION
    EXPECT_TRUE(v->GetType() == rendering::Visual::VT_COLLISION);

    // Check that the inertia visuals were properly added
    if (modelName != "new_box")
      v = scene->GetVisual(modelName + "::link_INERTIA_VISUAL__");
    else
      v = scene->GetVisual(modelName + "::body_INERTIA_VISUAL__");

    ASSERT_TRUE(v != nullptr);

    // Verify type is VT_PHYSICS
    EXPECT_TRUE(v->GetType() == rendering::Visual::VT_PHYSICS);

    // Check that the COM visuals were properly added
    if (modelName != "new_box")
      v = scene->GetVisual(modelName + "::link_COM_VISUAL__");
    else
      v = scene->GetVisual(modelName + "::body_COM_VISUAL__");

    ASSERT_TRUE(v != nullptr);

    // Verify type is VT_PHYSICS
    EXPECT_TRUE(v->GetType() == rendering::Visual::VT_PHYSICS);

    // Check that the link frame visuals were properly added
    if (modelName != "new_box")
      v = scene->GetVisual(modelName + "::link_LINK_FRAME_VISUAL__");
    else
      v = scene->GetVisual(modelName + "::body_LINK_FRAME_VISUAL__");

    ASSERT_TRUE(v != nullptr);

    // Verify type is VT_PHYSICS
    EXPECT_TRUE(v->GetType() == rendering::Visual::VT_PHYSICS);
  }
}

/////////////////////////////////////////////////
TEST_F(Scene_TEST, Shadows)
{
  Load("worlds/shapes.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != nullptr);

  // Test API for enabling and disabling shadows
  EXPECT_TRUE(scene->ShadowsEnabled());

  scene->SetShadowsEnabled(false);
  EXPECT_FALSE(scene->ShadowsEnabled());

  scene->SetShadowsEnabled(true);
  EXPECT_TRUE(scene->ShadowsEnabled());

  // Test API for updating shadow texture size
  EXPECT_GT(scene->ShadowTextureSize(), 0u);

  EXPECT_TRUE(scene->SetShadowTextureSize(256u));
  EXPECT_EQ(256u, scene->ShadowTextureSize());
  EXPECT_TRUE(scene->ShadowsEnabled());

  // setting a shadow texture size of 0 should not work
  EXPECT_FALSE(scene->SetShadowTextureSize(0u));
  EXPECT_EQ(256u, scene->ShadowTextureSize());
  EXPECT_TRUE(scene->ShadowsEnabled());
}

/////////////////////////////////////////////////
TEST_F(Scene_TEST, AddRemoveLights)
{
  Load("worlds/empty.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != nullptr);

  // Check that the scene has no lights
  // Note, the sun is not added yet unless we explicity trigger the preRender
  // event to process the messages from the server.
  EXPECT_EQ(scene->LightCount(), 0u);

  // Add a visual and check that it has been added
  rendering::LightPtr light1;
  light1.reset(new rendering::Light(scene));
  light1->SetName("light1");
  scene->AddLight(light1);
  EXPECT_EQ(scene->LightCount(), 1u);
  EXPECT_EQ(light1, scene->LightByName("light1"));
  EXPECT_EQ(light1, scene->LightById(light1->Id()));

  // Add anaother light and check that it has been added
  rendering::LightPtr light2;
  light2.reset(new rendering::Light(scene));
  light2->SetName("light2");
  scene->AddLight(light2);
  EXPECT_EQ(scene->LightCount(), 2u);
  EXPECT_EQ(light2, scene->LightByName("light2"));
  EXPECT_EQ(light2, scene->LightById(light2->Id()));

  // Remove a light and check that it has been removed
  scene->RemoveLight(light1);
  EXPECT_EQ(scene->LightCount(), 1u);
  EXPECT_FALSE(scene->LightByName("light1"));
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
