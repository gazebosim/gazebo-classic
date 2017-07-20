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

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ignition/math/Box.hh>

#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/InertiaVisual.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class InertiaVisual_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(InertiaVisual_TEST, InertiaVisualTest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
      scene = gazebo::rendering::create_scene("default", false);

  ASSERT_NE(scene, nullptr);

  ASSERT_NE(scene->WorldVisual(), nullptr);
  unsigned int count = scene->WorldVisual()->GetChildCount();

  // test calling constructor and Load functions and make sure
  // there are no segfaults
  gazebo::rendering::VisualPtr inertiaVis(new gazebo::rendering::InertiaVisual(
      "inertia_vis", scene->WorldVisual()));
  inertiaVis->Load();

  EXPECT_EQ(inertiaVis->GetName(), "inertia_vis");
  EXPECT_GT(scene->WorldVisual()->GetChildCount(), count);
  EXPECT_NE(scene->GetVisual(inertiaVis->GetName()), nullptr);

  // test destroying the visual
  inertiaVis->Fini();
  EXPECT_EQ(inertiaVis->GetChildCount(), 0u);

  // verify scene's child count is the same as before the visual was created
  EXPECT_EQ(scene->WorldVisual()->GetChildCount(), count);
}

/////////////////////////////////////////////////
TEST_F(InertiaVisual_TEST, InertiaRotation)
{
  Load("worlds/inertia_rotations.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  const std::vector<std::string> inertiaVisualNames =
  {
    "box010409_ref::link_INERTIA_VISUAL__",
    "box010409_x90::link_INERTIA_VISUAL__",
    "box010409_y90::link_INERTIA_VISUAL__",
    "box010409_z90::link_INERTIA_VISUAL__",
    "box010409_x45::link_INERTIA_VISUAL__",
    "box010409_y45::link_INERTIA_VISUAL__",
    "box010409_z45::link_INERTIA_VISUAL__"
  };

  // Generate inertia visuals
  scene->ShowInertias(true);

  // Wait until all models are inserted
  int sleep = 0;
  const int maxSleep = 50;
  while (sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    bool found = true;
    for (const auto name : inertiaVisualNames)
    {
      auto visual = scene->GetVisual(name);
      if (visual == nullptr)
      {
        found = false;
        break;
      }
    }
    if (found)
    {
      break;
    }
    common::Time::MSleep(200);
    sleep++;
  }
  EXPECT_LT(sleep, maxSleep);

  // expect bounding box of size 0.1 x 0.4 x 0.9
  const ignition::math::Box box(-0.05, -0.2, -0.45,
                                 0.05,  0.2,  0.45);
  for (const auto name : inertiaVisualNames)
  {
    gzdbg << "Check bounding box for "
          << name
          << std::endl;
    auto visual = scene->GetVisual(name);
    ASSERT_NE(visual, nullptr);
    // need to set these flags in order to GetBoundingBox
    visual->SetVisible(true);
    visual->SetVisibilityFlags(GZ_VISIBILITY_ALL);
    EXPECT_EQ(visual->GetBoundingBox().Ign(), box);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
