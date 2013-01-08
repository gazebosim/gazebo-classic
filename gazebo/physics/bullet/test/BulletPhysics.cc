/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <LinearMath/btVector3.h>
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "test_config.h"

#define NEAR_TOL 1e-6

using namespace gazebo;

/////////////////////////////////////////////////
/// Check Vector3 conversions
TEST(BulletPhysics, ConvertVector3)
{
  {
    math::Vector3 vec;
    btVector3 bt = physics::BulletPhysics::ConvertVector3(vec);
    EXPECT_EQ(bt.getX(), 0);
    EXPECT_EQ(bt.getY(), 0);
    EXPECT_EQ(bt.getZ(), 0);
  }

  {
    btVector3 bt;
    math::Vector3 vec = physics::BulletPhysics::ConvertVector3(bt);
    EXPECT_EQ(vec.x, 0);
    EXPECT_EQ(vec.y, 0);
    EXPECT_EQ(vec.z, 0);
  }

  {
    math::Vector3 vec(1, 2, 3);
    btVector3 bt = physics::BulletPhysics::ConvertVector3(vec);
    EXPECT_NEAR(bt.getX(), 1.0, NEAR_TOL);
    EXPECT_NEAR(bt.getY(), 2.0, NEAR_TOL);
    EXPECT_NEAR(bt.getZ(), 3.0, NEAR_TOL);
  }

  {
    btVector3 bt(1, 2, 3);
    math::Vector3 vec = physics::BulletPhysics::ConvertVector3(bt);
    EXPECT_NEAR(vec.x, 1.0, NEAR_TOL);
    EXPECT_NEAR(vec.y, 2.0, NEAR_TOL);
    EXPECT_NEAR(vec.z, 3.0, NEAR_TOL);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
