/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "gazebo/rendering/Visual.hh"
#include "test/ServerFixture.hh"


using namespace gazebo;
class Visual_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(Visual_TEST, BoundingBox)
{
  Load("worlds/empty.world");

  // Spawn a box and check its bounding box dimensions.
  SpawnBox("box", math::Vector3(1, 1, 1), math::Vector3(10, 10, 1),
      math::Vector3(0, 0, 0));

  // FIXME need a camera otherwise test produces a gl vertex buffer error
  math::Pose cameraStartPose(0, 0, 0, 0, 0, 0);
  std::string cameraName = "test_camera";
  SpawnCamera("test_camera_model", cameraName,
      cameraStartPose.pos, cameraStartPose.rot.GetAsEuler());

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  int sleep = 0;
  int maxSleep = 50;
  rendering::VisualPtr visual;
  while (!visual && sleep < maxSleep)
  {
    visual = scene->GetVisual("box");
    common::Time::MSleep(1000);
    sleep++;
  }
  ASSERT_TRUE(visual != NULL);

  // verify initial bounding box
  math::Vector3 bboxMin(-0.5, -0.5, -0.5);
  math::Vector3 bboxMax(0.5, 0.5, 0.5);
  math::Box boundingBox = visual->GetBoundingBox();
  EXPECT_EQ(boundingBox.min, bboxMin);
  EXPECT_EQ(boundingBox.max, bboxMax);

  // verify scale
  math::Vector3 scale = visual->GetScale();
  EXPECT_EQ(scale, math::Vector3(1, 1, 1));

  // set new scale
  math::Vector3 scaleToSet(2, 3, 4);
  visual->SetScale(scaleToSet);

  // verify new scale
  math::Vector3 newScale = visual->GetScale();
  EXPECT_EQ(newScale, scaleToSet);
  EXPECT_EQ(newScale, math::Vector3(2, 3, 4));

  // verify local bounding box dimensions remain the same
  math::Box newBoundingBox = visual->GetBoundingBox();
  EXPECT_EQ(newBoundingBox.min, bboxMin);
  EXPECT_EQ(newBoundingBox.max, bboxMax);

  // verify local bounding box dimensions with scale applied
  EXPECT_EQ(newScale*newBoundingBox.min, newScale*bboxMin);
  EXPECT_EQ(newScale*newBoundingBox.max, newScale*bboxMax);
  EXPECT_EQ(newScale*newBoundingBox.min, math::Vector3(-1, -1.5, -2));
  EXPECT_EQ(newScale*newBoundingBox.max, math::Vector3(1, 1.5, 2));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
