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
//#include "gazebo/math/Rand.hh"
//#include "gazebo/rendering/RenderingIface.hh"
//#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Scene.hh"
#include "test/ServerFixture.hh"


using namespace gazebo;
class Scene_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(Scene_TEST, AddRemoveVisuals)
{
  Load("worlds/empty.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  // Check that it only has one visual, the world visual
  EXPECT_EQ(scene->GetVisualCount(), 1);
  EXPECT_TRUE(scene->GetVisual("__world_node__") != NULL);

  // Add a visual and check that it has been added
  rendering::VisualPtr visual1;
  visual1.reset(new rendering::Visual("visual1", scene));
  scene->AddVisual(visual1);
  EXPECT_EQ(scene->GetVisualCount(), 2);
  EXPECT_TRUE(scene->GetVisual("visual1") != NULL);

  // Add a visual and check that it has been added
  rendering::VisualPtr visual2;
  visual2.reset(new rendering::Visual("visual2", scene));
  scene->AddVisual(visual2);
  EXPECT_EQ(scene->GetVisualCount(), 3);
  EXPECT_TRUE(scene->GetVisual("visual2") != NULL);

  // Remove a visual and check that it has been removed
  scene->RemoveVisual(visual1);
  EXPECT_EQ(scene->GetVisualCount(), 2);
  EXPECT_FALSE(scene->GetVisual("visual1"));
}

/////////////////////////////////////////////////
TEST_F(Scene_TEST, RemoveModelVisual)
{
  // Load a world containing 3 simple shapes
  Load("worlds/shapes.world");

  // FIXME need a camera otherwise test produces a gl vertex buffer error
  math::Pose cameraStartPose(0, 0, 0, 0, 0, 0);
  std::string cameraName = "test_camera";
  SpawnCamera("test_camera_model", cameraName,
      cameraStartPose.pos, cameraStartPose.rot.GetAsEuler());

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  // Wait until all models are inserted
  int sleep = 0;
  int maxSleep = 10;
  rendering::VisualPtr box, sphere, cylinder;
  while ((!box || !sphere || !cylinder) && sleep < maxSleep)
  {
    box = scene->GetVisual("box");
    cylinder = scene->GetVisual("cylinder");
    sphere = scene->GetVisual("sphere");
    common::Time::MSleep(1000);
    sleep++;
  }

  // Check that the model visuals were properly added
  ASSERT_TRUE(box != NULL);
  ASSERT_TRUE(sphere != NULL);
  ASSERT_TRUE(cylinder != NULL);

  // Check that the link visuals were properly added
  EXPECT_TRUE(scene->GetVisual("box::link") != NULL);
  EXPECT_TRUE(scene->GetVisual("cylinder::link") != NULL);
  EXPECT_TRUE(scene->GetVisual("sphere::link") != NULL);

  // Check that the "visual visuals" were properly added
  EXPECT_TRUE(scene->GetVisual("box::link::visual") != NULL);
  EXPECT_TRUE(scene->GetVisual("cylinder::link::visual") != NULL);
  EXPECT_TRUE(scene->GetVisual("sphere::link::visual") != NULL);

  // Check that the collision visuals were properly added
  EXPECT_TRUE(scene->GetVisual("box::link::collision") != NULL);
  EXPECT_TRUE(scene->GetVisual("cylinder::link::collision") != NULL);
  EXPECT_TRUE(scene->GetVisual("sphere::link::collision") != NULL);

  // Check that the inertia visuals were properly added
  EXPECT_TRUE(scene->GetVisual("box::link_INERTIA_VISUAL__") != NULL);
  EXPECT_TRUE(scene->GetVisual("cylinder::link_INERTIA_VISUAL__") != NULL);
  EXPECT_TRUE(scene->GetVisual("sphere::link_INERTIA_VISUAL__") != NULL);

  // Check that the COM visuals were properly added
  EXPECT_TRUE(scene->GetVisual("box::link_COM_VISUAL__") != NULL);
  EXPECT_TRUE(scene->GetVisual("cylinder::link_COM_VISUAL__") != NULL);
  EXPECT_TRUE(scene->GetVisual("sphere::link_COM_VISUAL__") != NULL);

// Check the default::box* as well?

  // Send request to delete the box model
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  transport::requestNoReply(node, "entity_delete", "box");

  sleep = 0;
  while (box && sleep < maxSleep)
  {
    box = scene->GetVisual("box");
    common::Time::MSleep(1000);
    sleep++;
  }
  ASSERT_TRUE(box == NULL);

  // Check that the link visuals were properly added
  EXPECT_TRUE(scene->GetVisual("box::link") == NULL);

  // Check that the "visual visuals" were properly added
  EXPECT_TRUE(scene->GetVisual("box::link::visual") == NULL);

  // Check that the collision visuals were properly added
  EXPECT_TRUE(scene->GetVisual("box::link::collision") == NULL);

  // Check that the inertia visuals were properly added
  EXPECT_TRUE(scene->GetVisual("box::link_INERTIA_VISUAL__") == NULL);

  // Check that the COM visuals were properly added
  EXPECT_TRUE(scene->GetVisual("box::link_COM_VISUAL__") == NULL);



  scene->PrintSceneGraph();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
