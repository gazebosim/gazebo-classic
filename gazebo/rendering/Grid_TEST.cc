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
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Grid.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class Grid_TEST : public RenderingFixture
{
};

//////////////////////////////////////////////////
TEST_F(Grid_TEST, SetSize)
{
  this->Load("worlds/empty.world");

  auto scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != nullptr);

  // Create a grid
  int cellCount = 10;
  float cellLength  = 1;
  float lineWidth = 0.1;
  auto grid = new gazebo::rendering::Grid(scene.get(), cellCount, cellLength,
      lineWidth, common::Color::Red);
  ASSERT_TRUE(grid != nullptr);

  grid->Init();

  // Get scene node and manual object
  rendering::VisualPtr vis = grid->GridVisual();
  ASSERT_TRUE(vis != nullptr);
  auto sceneNode = vis->GetSceneNode();
  ASSERT_TRUE(sceneNode != nullptr);
  EXPECT_EQ(sceneNode->numAttachedObjects(), 1u);

  auto manualObject = sceneNode->getAttachedObject(0);
  ASSERT_TRUE(manualObject != nullptr);

  // Check size
  EXPECT_EQ(manualObject->getBoundingBox(),
      Ogre::AxisAlignedBox(-cellCount * cellLength / 2,
                           -cellCount * cellLength / 2,
                           0.015,
                           cellCount * cellLength / 2,
                           cellCount * cellLength / 2,
                           0.015));

  // Various sizes
  std::vector<int> cellCountList = {1, 33, 100, 40};
  std::vector<float> cellLengthList = {0.001, 0.4, 9.3, 1000};

  for (auto count : cellCountList)
  {
    grid->SetCellCount(count);
    for (auto length : cellLengthList)
    {
      grid->SetCellLength(length);

      gzmsg << "Checking count [" << count << "] length [" << length << "]"
          << std::endl;

      // Check size
      EXPECT_EQ(manualObject->getBoundingBox(),
          Ogre::AxisAlignedBox(-count * length / 2,
                               -count * length / 2,
                               0.015,
                               count * length / 2,
                               count * length / 2,
                               0.015));
    }
  }

  delete grid;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
