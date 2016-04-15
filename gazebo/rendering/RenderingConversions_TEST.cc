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

#include <ignition/math/Matrix4.hh>

#include "test/util.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Conversions.hh"

using namespace gazebo;
class Conversions_TEST : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(Conversions_TEST, Matrix4)
{
  // identity
  EXPECT_EQ(rendering::Conversions::Convert(Ogre::Matrix4::IDENTITY),
      ignition::math::Matrix4d::Identity);
  EXPECT_EQ(rendering::Conversions::Convert(ignition::math::Matrix4d::Identity),
      Ogre::Matrix4::IDENTITY);

  // zero
  EXPECT_EQ(rendering::Conversions::Convert(Ogre::Matrix4::ZERO),
      ignition::math::Matrix4d::Zero);
  EXPECT_EQ(rendering::Conversions::Convert(ignition::math::Matrix4d::Zero),
      Ogre::Matrix4::ZERO);

  // rotation and translation
  Ogre::Matrix4 m1(1, 0, 0, 2,
                   0, 0, -1, 3,
                   0, 1, 0, -5,
                   0, 0, 0, 1);

  ignition::math::Matrix4d m2(1, 0, 0, 2,
                              0, 0, -1, 3,
                              0, 1, 0, -5,
                              0, 0, 0, 1);
  EXPECT_EQ(rendering::Conversions::Convert(m1), m2);
  EXPECT_EQ(rendering::Conversions::Convert(m2), m1);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
