/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "test_config.h"
#include "test/util.hh"

#define NEAR_TOL 2e-5

using namespace gazebo;

class BulletTypes : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
/// Check Vector3 conversions
TEST_F(BulletTypes, ConvertVector3)
{
  {
    math::Vector3 vec, vec2;
    btVector3 bt = physics::BulletTypes::ConvertVector3(vec);
    EXPECT_NEAR(bt.getX(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getY(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getZ(), 0, NEAR_TOL);
    vec2 = physics::BulletTypes::ConvertVector3(bt);
    EXPECT_LT((vec-vec2).GetSquaredLength(), NEAR_TOL*NEAR_TOL);
  }

  {
    math::Vector3 vec(100.5, -2.314, 42), vec2;
    btVector3 bt = physics::BulletTypes::ConvertVector3(vec);
    EXPECT_NEAR(bt.getX(), vec.x, NEAR_TOL);
    EXPECT_NEAR(bt.getY(), vec.y, NEAR_TOL);
    EXPECT_NEAR(bt.getZ(), vec.z, NEAR_TOL);
    vec2 = physics::BulletTypes::ConvertVector3(bt);
    EXPECT_LT((vec-vec2).GetSquaredLength(), NEAR_TOL*NEAR_TOL);
  }
}

/////////////////////////////////////////////////
/// Check Vector4 conversions
TEST_F(BulletTypes, ConvertVector4)
{
  {
    math::Vector4 vec, vec2;
    btVector4 bt = physics::BulletTypes::ConvertVector4(vec);
    EXPECT_NEAR(bt.getX(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getY(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getZ(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getW(), 0, NEAR_TOL);
    vec2 = physics::BulletTypes::ConvertVector4(bt);
    EXPECT_LT((vec-vec2).GetSquaredLength(), NEAR_TOL*NEAR_TOL);
  }

  {
    math::Vector4 vec(100.5, -2.314, 42, 848.8), vec2;
    btVector4 bt = physics::BulletTypes::ConvertVector4(vec);
    EXPECT_NEAR(bt.getX(), vec.x, NEAR_TOL);
    EXPECT_NEAR(bt.getY(), vec.y, NEAR_TOL);
    EXPECT_NEAR(bt.getZ(), vec.z, NEAR_TOL);
    EXPECT_NEAR(bt.getW(), vec.w, NEAR_TOL);
    vec2 = physics::BulletTypes::ConvertVector4(bt);
    EXPECT_LT((vec-vec2).GetSquaredLength(), NEAR_TOL*NEAR_TOL);
  }
}

/////////////////////////////////////////////////
/// Check Pose conversions
TEST_F(BulletTypes, ConvertPose)
{
  {
    math::Pose pose, pose2;
    btTransform bt = physics::BulletTypes::ConvertPose(pose);
    EXPECT_NEAR(bt.getOrigin().getX(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getOrigin().getY(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getOrigin().getZ(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getX(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getY(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getZ(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getW(), 1, NEAR_TOL);
    pose2 = physics::BulletTypes::ConvertPose(bt);
    EXPECT_LT((pose.pos-pose2.pos).GetSquaredLength(), NEAR_TOL*NEAR_TOL);
    EXPECT_LT((pose.rot-pose2.rot).x, NEAR_TOL);
    EXPECT_LT((pose.rot-pose2.rot).y, NEAR_TOL);
    EXPECT_LT((pose.rot-pose2.rot).z, NEAR_TOL);
    EXPECT_LT((pose.rot-pose2.rot).w, NEAR_TOL);
  }

  {
    math::Pose pose(105.4, 3.1, -0.34, 3.14/8, 3.14/16, -3.14/2), pose2;
    btTransform bt = physics::BulletTypes::ConvertPose(pose);
    EXPECT_NEAR(bt.getOrigin().getX(), pose.pos.x, NEAR_TOL);
    EXPECT_NEAR(bt.getOrigin().getY(), pose.pos.y, NEAR_TOL);
    EXPECT_NEAR(bt.getOrigin().getZ(), pose.pos.z, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getX(), pose.rot.x, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getY(), pose.rot.y, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getZ(), pose.rot.z, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getW(), pose.rot.w, NEAR_TOL);
    pose2 = physics::BulletTypes::ConvertPose(bt);
    EXPECT_LT((pose.pos-pose2.pos).GetSquaredLength(), NEAR_TOL*NEAR_TOL);
    EXPECT_LT((pose.rot-pose2.rot).x, NEAR_TOL);
    EXPECT_LT((pose.rot-pose2.rot).y, NEAR_TOL);
    EXPECT_LT((pose.rot-pose2.rot).z, NEAR_TOL);
    EXPECT_LT((pose.rot-pose2.rot).w, NEAR_TOL);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
