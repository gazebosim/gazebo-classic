/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Conversions.hh"

using namespace gazebo;
class Conversions_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(Conversions_TEST, TransformSpace)
{
  EXPECT_EQ(rendering::Conversions::Convert(Ogre::Node::TS_LOCAL),
      rendering::RF_LOCAL);
  EXPECT_EQ(rendering::Conversions::Convert(Ogre::Node::TS_PARENT),
      rendering::RF_PARENT);
  EXPECT_EQ(rendering::Conversions::Convert(Ogre::Node::TS_WORLD),
      rendering::RF_WORLD);

  EXPECT_EQ(rendering::Conversions::Convert(rendering::RF_LOCAL),
      Ogre::Node::TS_LOCAL);
  EXPECT_EQ(rendering::Conversions::Convert(rendering::RF_PARENT),
      Ogre::Node::TS_PARENT);
  EXPECT_EQ(rendering::Conversions::Convert(rendering::RF_WORLD),
      Ogre::Node::TS_WORLD);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
