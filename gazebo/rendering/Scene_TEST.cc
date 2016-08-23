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

  // Check that it has two visuals, the world and origin visuals
  EXPECT_EQ(scene->GetVisualCount(), 2u);
  EXPECT_TRUE(scene->GetVisual("__world_node__") != NULL);

  // Add a visual and check that it has been added
  rendering::VisualPtr visual1;
  visual1.reset(new rendering::Visual("visual1", scene));
  scene->AddVisual(visual1);
  EXPECT_EQ(scene->GetVisualCount(), 3u);
  EXPECT_TRUE(scene->GetVisual("visual1") != NULL);

  // Add a visual and check that it has been added
  rendering::VisualPtr visual2;
  visual2.reset(new rendering::Visual("visual2", scene));
  scene->AddVisual(visual2);
  EXPECT_EQ(scene->GetVisualCount(), 4u);
  EXPECT_TRUE(scene->GetVisual("visual2") != NULL);

  // Remove a visual and check that it has been removed
  scene->RemoveVisual(visual1);
  EXPECT_EQ(scene->GetVisualCount(), 3u);
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

  // Check that the link frame visuals were properly added
  EXPECT_TRUE(scene->GetVisual("box::link_LINK_FRAME_VISUAL__") != NULL);
  EXPECT_TRUE(scene->GetVisual("cylinder::link_LINK_FRAME_VISUAL__") != NULL);
  EXPECT_TRUE(scene->GetVisual("sphere::link_LINK_FRAME_VISUAL__") != NULL);

  // Send request to delete the box model
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::requestNoReply(node, "entity_delete", "box");

  sleep = 0;
  while (box && sleep < maxSleep)
  {
    box = scene->GetVisual("box");
    common::Time::MSleep(1000);
    sleep++;
  }
  ASSERT_TRUE(box == NULL);

  // Check that the link visuals were properly removed
  EXPECT_TRUE(scene->GetVisual("box::link") == NULL);

  // Check that the "visual visuals" were properly removed
  EXPECT_TRUE(scene->GetVisual("box::link::visual") == NULL);

  // Check that the collision visuals were properly removed
  EXPECT_TRUE(scene->GetVisual("box::link::collision") == NULL);

  // Check that the inertia visuals were properly removed
  EXPECT_TRUE(scene->GetVisual("box::link_INERTIA_VISUAL__") == NULL);

  // Check that the COM visuals were properly removed
  EXPECT_TRUE(scene->GetVisual("box::link_COM_VISUAL__") == NULL);

  // Check that the link frame visuals were properly removed
  EXPECT_TRUE(scene->GetVisual("box::link_LINK_FRAME_VISUAL__") == NULL);
}

/////////////////////////////////////////////////
TEST_F(Scene_TEST, VisualType)
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

  // Spawn another box. The model loading process is slightly different so
  // check that its visuals also have the correct visual type.
  SpawnBox("new_box", math::Vector3(1, 1, 1), math::Vector3(10, 10, 1),
      math::Vector3(0, 0, 0));

  // Wait until all models are inserted
  int sleep = 0;
  int maxSleep = 10;
  rendering::VisualPtr box, sphere, cylinder, newBox;
  while ((!box || !sphere || !cylinder || !newBox) && sleep < maxSleep)
  {
    box = scene->GetVisual("box");
    cylinder = scene->GetVisual("cylinder");
    sphere = scene->GetVisual("sphere");
    newBox = scene->GetVisual("new_box");
    common::Time::MSleep(1000);
    sleep++;
  }

  // Check that the model visuals were properly added
  ASSERT_TRUE(box != NULL);
  ASSERT_TRUE(sphere != NULL);
  ASSERT_TRUE(cylinder != NULL);
  ASSERT_TRUE(newBox != NULL);

  // Verify type is VT_MODEL
  EXPECT_TRUE(box->GetType() == rendering::Visual::VT_MODEL);
  EXPECT_TRUE(sphere->GetType() == rendering::Visual::VT_MODEL);
  EXPECT_TRUE(cylinder->GetType() == rendering::Visual::VT_MODEL);
  EXPECT_TRUE(newBox->GetType() == rendering::Visual::VT_MODEL);

  // Check that the link visuals were properly added
  rendering::VisualPtr boxLink = scene->GetVisual("box::link");
  rendering::VisualPtr sphereLink = scene->GetVisual("sphere::link");
  rendering::VisualPtr cylinderLink = scene->GetVisual("cylinder::link");
  rendering::VisualPtr newBoxLink = scene->GetVisual("new_box::body");
  EXPECT_TRUE(boxLink != NULL);
  EXPECT_TRUE(sphereLink != NULL);
  EXPECT_TRUE(cylinderLink != NULL);
  EXPECT_TRUE(newBoxLink != NULL);

  // Verify type is VT_LINK
  EXPECT_TRUE(boxLink->GetType() == rendering::Visual::VT_LINK);
  EXPECT_TRUE(sphereLink->GetType() == rendering::Visual::VT_LINK);
  EXPECT_TRUE(cylinderLink->GetType() == rendering::Visual::VT_LINK);
  EXPECT_TRUE(newBoxLink->GetType() == rendering::Visual::VT_LINK);

  // Check that the visual visuals were properly added
  rendering::VisualPtr boxVisual = scene->GetVisual("box::link::visual");
  rendering::VisualPtr sphereVisual = scene->GetVisual("sphere::link::visual");
  rendering::VisualPtr cylinderVisual =
      scene->GetVisual("cylinder::link::visual");
  rendering::VisualPtr newBoxVisual = scene->GetVisual("new_box::body::visual");
  EXPECT_TRUE(boxVisual != NULL);
  EXPECT_TRUE(sphereVisual != NULL);
  EXPECT_TRUE(cylinderVisual != NULL);
  EXPECT_TRUE(newBoxVisual != NULL);

  // Verify type is VT_VISUAL
  EXPECT_TRUE(boxVisual->GetType() == rendering::Visual::VT_VISUAL);
  EXPECT_TRUE(sphereVisual->GetType() == rendering::Visual::VT_VISUAL);
  EXPECT_TRUE(cylinderVisual->GetType() == rendering::Visual::VT_VISUAL);
  EXPECT_TRUE(newBoxVisual->GetType() == rendering::Visual::VT_VISUAL);

  // Check that the collision visuals were properly added
  rendering::VisualPtr boxCollision = scene->GetVisual("box::link::collision");
  rendering::VisualPtr sphereCollision =
      scene->GetVisual("sphere::link::collision");
  rendering::VisualPtr cylinderCollision =
      scene->GetVisual("cylinder::link::collision");
  rendering::VisualPtr newBoxCollision =
      scene->GetVisual("new_box::body::geom");
  EXPECT_TRUE(boxCollision != NULL);
  EXPECT_TRUE(sphereCollision != NULL);
  EXPECT_TRUE(cylinderCollision != NULL);
  EXPECT_TRUE(newBoxCollision != NULL);

  // Verify type is VT_COLLISION
  EXPECT_TRUE(boxCollision->GetType() == rendering::Visual::VT_COLLISION);
  EXPECT_TRUE(sphereCollision->GetType() == rendering::Visual::VT_COLLISION);
  EXPECT_TRUE(cylinderCollision->GetType() == rendering::Visual::VT_COLLISION);
  EXPECT_TRUE(newBoxCollision->GetType() == rendering::Visual::VT_COLLISION);

  // Check that the inertia visuals were properly added
  rendering::VisualPtr boxInertia =
      scene->GetVisual("box::link_INERTIA_VISUAL__");
  rendering::VisualPtr sphereInertia =
      scene->GetVisual("sphere::link_INERTIA_VISUAL__");
  rendering::VisualPtr cylinderInertia =
      scene->GetVisual("cylinder::link_INERTIA_VISUAL__");
  rendering::VisualPtr newBoxInertia =
      scene->GetVisual("new_box::body_INERTIA_VISUAL__");
  EXPECT_TRUE(boxInertia != NULL);
  EXPECT_TRUE(sphereInertia != NULL);
  EXPECT_TRUE(cylinderInertia != NULL);
  EXPECT_TRUE(newBoxInertia != NULL);

  // Verify type is VT_PHYSICS
  EXPECT_TRUE(boxInertia->GetType() == rendering::Visual::VT_PHYSICS);
  EXPECT_TRUE(sphereInertia->GetType() == rendering::Visual::VT_PHYSICS);
  EXPECT_TRUE(cylinderInertia->GetType() == rendering::Visual::VT_PHYSICS);
  EXPECT_TRUE(newBoxInertia->GetType() == rendering::Visual::VT_PHYSICS);

  // Check that the COM visuals were properly added
  rendering::VisualPtr boxCOM = scene->GetVisual("box::link_COM_VISUAL__");
  rendering::VisualPtr sphereCOM =
      scene->GetVisual("sphere::link_COM_VISUAL__");
  rendering::VisualPtr cylinderCOM =
      scene->GetVisual("cylinder::link_COM_VISUAL__");
  rendering::VisualPtr newBoxCOM =
      scene->GetVisual("new_box::body_COM_VISUAL__");
  EXPECT_TRUE(boxCOM != NULL);
  EXPECT_TRUE(sphereCOM != NULL);
  EXPECT_TRUE(cylinderCOM != NULL);
  EXPECT_TRUE(newBoxCOM != NULL);

  // Verify type is VT_PHYSICS
  EXPECT_TRUE(boxCOM->GetType() == rendering::Visual::VT_PHYSICS);
  EXPECT_TRUE(cylinderCOM->GetType() == rendering::Visual::VT_PHYSICS);
  EXPECT_TRUE(cylinderCOM->GetType() == rendering::Visual::VT_PHYSICS);
  EXPECT_TRUE(newBoxCOM->GetType() == rendering::Visual::VT_PHYSICS);

  // Check that the link frame visuals were properly added
  rendering::VisualPtr boxLinkFrame =
      scene->GetVisual("box::link_LINK_FRAME_VISUAL__");
  rendering::VisualPtr sphereLinkFrame =
      scene->GetVisual("sphere::link_LINK_FRAME_VISUAL__");
  rendering::VisualPtr cylinderLinkFrame =
      scene->GetVisual("cylinder::link_LINK_FRAME_VISUAL__");
  rendering::VisualPtr newBoxLinkFrame =
      scene->GetVisual("new_box::body_LINK_FRAME_VISUAL__");
  EXPECT_TRUE(boxLinkFrame != NULL);
  EXPECT_TRUE(sphereLinkFrame != NULL);
  EXPECT_TRUE(cylinderLinkFrame != NULL);
  EXPECT_TRUE(newBoxLinkFrame != NULL);

  // Verify type is VT_PHYSICS
  EXPECT_TRUE(boxLinkFrame->GetType() == rendering::Visual::VT_PHYSICS);
  EXPECT_TRUE(cylinderLinkFrame->GetType() == rendering::Visual::VT_PHYSICS);
  EXPECT_TRUE(cylinderLinkFrame->GetType() == rendering::Visual::VT_PHYSICS);
  EXPECT_TRUE(newBoxLinkFrame->GetType() == rendering::Visual::VT_PHYSICS);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
