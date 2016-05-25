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

#include <ignition/math/Matrix4.hh>

#include "test/util.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Conversions.hh"

using namespace gazebo;
class Conversions_TEST : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(Conversions_TEST, Matrix4)
{
  // identity
  EXPECT_EQ(rendering::Conversions::ConvertIgn(Ogre::Matrix4::IDENTITY),
      ignition::math::Matrix4d::Identity);
  EXPECT_EQ(rendering::Conversions::Convert(ignition::math::Matrix4d::Identity),
      Ogre::Matrix4::IDENTITY);

  // zero
  EXPECT_EQ(rendering::Conversions::ConvertIgn(Ogre::Matrix4::ZERO),
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
  EXPECT_EQ(rendering::Conversions::ConvertIgn(m1), m2);
  EXPECT_EQ(rendering::Conversions::Convert(m2), m1);
}

/////////////////////////////////////////////////
TEST_F(Conversions_TEST, Quaternion)
{
  // identity
  EXPECT_EQ(rendering::Conversions::ConvertIgn(Ogre::Quaternion::IDENTITY),
      ignition::math::Quaterniond::Identity);
  EXPECT_EQ(rendering::Conversions::Convert(
      ignition::math::Quaterniond::Identity), Ogre::Quaternion::IDENTITY);

  // zero
  EXPECT_EQ(rendering::Conversions::ConvertIgn(Ogre::Quaternion::ZERO),
      ignition::math::Quaterniond(0, 0, 0, 0));
  EXPECT_EQ(rendering::Conversions::Convert(
      ignition::math::Quaterniond(0, 0, 0, 0)), Ogre::Quaternion::ZERO);

  // rotation
  Ogre::Quaternion q1(0.7071, 0, 0.7071, 0);
  ignition::math::Quaterniond q2(0.7071, 0, 0.7071, 0);

  EXPECT_EQ(rendering::Conversions::ConvertIgn(q1), q2);
  EXPECT_EQ(rendering::Conversions::Convert(q2), q1);
}

/////////////////////////////////////////////////
TEST_F(Conversions_TEST, Vector3)
{
  // one
  EXPECT_EQ(rendering::Conversions::ConvertIgn(Ogre::Vector3::UNIT_SCALE),
      ignition::math::Vector3d::One);
  EXPECT_EQ(rendering::Conversions::Convert(
      ignition::math::Vector3d::One), Ogre::Vector3::UNIT_SCALE);

  // zero
  EXPECT_EQ(rendering::Conversions::ConvertIgn(Ogre::Vector3::ZERO),
      ignition::math::Vector3d::Zero);
  EXPECT_EQ(rendering::Conversions::Convert(ignition::math::Vector3d::Zero),
      Ogre::Vector3::ZERO);

  // random vector3 values
  Ogre::Vector3 v1(0.3, -50, 8);
  ignition::math::Vector3d v2(0.3, -50, 8);

  EXPECT_EQ(rendering::Conversions::ConvertIgn(v1), v2);
  EXPECT_EQ(rendering::Conversions::Convert(v2), v1);
}

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
