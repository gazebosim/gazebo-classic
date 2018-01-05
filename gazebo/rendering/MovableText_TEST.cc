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
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/MovableText.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
using namespace rendering;
class MovableText_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(MovableText_TEST, MovableTextTest)
{
  this->Load("worlds/empty.world");

  // Scene
  auto scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  ASSERT_NE(nullptr, scene);

  // World vis
  auto worldVis = scene->WorldVisual();
  ASSERT_NE(nullptr, worldVis);

  // Movable text
  MovableText text;
  text.Load("TEST_TEXT", "banana");

  // Visual to hold text
  VisualPtr vis(new Visual("TEST_TEXT_VIS", worldVis, false));
  ASSERT_NE(nullptr, vis);
  vis->Load();
  vis->AttachObject(&(text));
  vis->GetSceneNode()->setInheritScale(false);

  // Check default values
  EXPECT_EQ("banana", text.Text());
  EXPECT_EQ("Arial", text.FontName());
  EXPECT_EQ(ignition::math::Color::White, text.Color());
  EXPECT_FLOAT_EQ(1.0f, text.CharHeight());
  EXPECT_LT(1.0, text.SpaceWidth());
  EXPECT_FLOAT_EQ(0.0f, text.Baseline());
  EXPECT_FALSE(text.ShowOnTop());

  // Set text
  text.SetText("acerola");
  EXPECT_EQ("acerola", text.Text());

  text.SetText("");
  EXPECT_EQ("", text.Text());

  text.SetText("apples and oranges");
  EXPECT_EQ("apples and oranges", text.Text());

  // Set font
  EXPECT_THROW(text.SetFontName("banana"), Ogre::Exception);
  EXPECT_EQ("Arial", text.FontName());

  text.SetFontName("Console");
  EXPECT_EQ("Console", text.FontName());

  // Set color
  text.SetColor(ignition::math::Color::Red);
  EXPECT_EQ(ignition::math::Color::Red, text.Color());

  // Set character height
  text.SetCharHeight(0.0);
  EXPECT_FLOAT_EQ(0.0, text.CharHeight());

  text.SetCharHeight(-1.0);
  EXPECT_FLOAT_EQ(-1.0, text.CharHeight());

  text.SetCharHeight(0.5);
  EXPECT_FLOAT_EQ(0.5, text.CharHeight());

  // Set space width
  text.SetSpaceWidth(0.0);
  EXPECT_FLOAT_EQ(0.0, text.SpaceWidth());

  text.SetSpaceWidth(-1.0);
  EXPECT_FLOAT_EQ(-1.0, text.SpaceWidth());

  text.SetSpaceWidth(0.5);
  EXPECT_FLOAT_EQ(0.5, text.SpaceWidth());

  // Set baseline
  text.SetBaseline(0.5);
  EXPECT_FLOAT_EQ(0.5, text.Baseline());

  // Set show on top
  text.SetShowOnTop(true);
  EXPECT_TRUE(text.ShowOnTop());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
