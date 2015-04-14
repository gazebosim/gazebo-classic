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
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ApplyWrenchVisual.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;
class ApplyWrenchVisual_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(ApplyWrenchVisual_TEST, ApplyWrenchVisualTest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != NULL);

  // create a visual to attach the apply wrench visual to
  gazebo::rendering::VisualPtr linkVis;
  linkVis.reset(
      new gazebo::rendering::Visual("link", scene->GetWorldVisual()));
  EXPECT_TRUE(linkVis != NULL);

  // test calling constructor and Load functions and make sure
  // there are no segfaults
  gazebo::rendering::ApplyWrenchVisualPtr applyWrenchVis(
      new gazebo::rendering::ApplyWrenchVisual("apply_wrench_vis", linkVis));
  applyWrenchVis->Load();








  applyWrenchVis->Fini();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
