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
#include "gazebo/rendering/LogicalCameraVisual.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class LogicalCameraVisual_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(LogicalCameraVisual_TEST, LoadDestroy)
{
  this->Load("worlds/empty.world");

  auto scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);
  EXPECT_NE(scene, nullptr);

  // get scene visual child count before we create any visuals
  EXPECT_NE(scene->WorldVisual(), nullptr);

  unsigned int count = scene->WorldVisual()->GetChildCount();

  // test calling constructor and Load functions and make sure
  // there are no segfaults
  gazebo::rendering::VisualPtr vis(
      new gazebo::rendering::LogicalCameraVisual("test_vis",
      scene->WorldVisual()));
  vis->Load();

  // Check visual was created
  EXPECT_GT(scene->WorldVisual()->GetChildCount(), count);

  // test destroying the visual
  vis->Fini();
  vis.reset();

  // verify scene's child count is the same as before the visual was created
  EXPECT_EQ(scene->WorldVisual()->GetChildCount(), count);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
