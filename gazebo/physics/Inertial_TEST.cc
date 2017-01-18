/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include "gazebo/physics/Inertial.hh"
#include "test/util.hh"

#define TOL 1e-6
using namespace gazebo;

class Inertial_TEST : public gazebo::testing::AutoLogFixture { };

////////////////////////////////////////////////////////////////////////
// Test basic Inertial functions
////////////////////////////////////////////////////////////////////////
TEST_F(Inertial_TEST, InertialOperators)
{
  physics::Inertial i1;
  physics::Inertial i2;
  i1.SetMass(1.0);
  i2.SetMass(2.0);
  i1.SetCoG(ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  i2.SetCoG(ignition::math::Pose3d(0, 0, 1, 0, 0, 0));

  i1.SetIXX(0.1);
  i1.SetIYY(0.2);
  i1.SetIZZ(0.3);
  i1.SetIXY(0.0);
  i1.SetIXZ(0.0);
  i1.SetIYZ(0.0);

  i2.SetMOI(ignition::math::Matrix3d(1, 0, 0,
                                     0, 2, 0,
                                     0, 0, 3));

  // Test addition
  physics::Inertial isum = i1 + i2;
  gzdbg << "isum: \n"
        << isum << "\n";
  gzdbg << "i1 new cg: \n"
        << i1.MOI(isum.Pose()) << "\n";
  gzdbg << "i2 new cg: \n"
        << i2.MOI(isum.Pose()) << "\n";
  EXPECT_NEAR(isum.Pose().Pos().Z(), 2.0/3.0, TOL);
  EXPECT_NEAR(isum.IXX(),
    1.0 + 0.1 + 1.0*(2.0/3.0)*(2.0/3.0)
              + 2.0*(1-2.0/3.0)*(1-2.0/3.0), TOL);
  EXPECT_NEAR(isum.IYY(),
    2.0 + 0.2 + 1.0*(2.0/3.0)*(2.0/3.0)
              + 2.0*(1-2.0/3.0)*(1-2.0/3.0), TOL);
  EXPECT_NEAR(isum.IZZ(), 3.0 + 0.3, TOL);

  // Test GetInertial(offset)
  physics::Inertial i1Offset = i1(ignition::math::Pose3d::Zero);
  EXPECT_TRUE(i1.MOI() == i1Offset.MOI());
  EXPECT_TRUE(i1.Pose() == i1Offset.Pose());
  i1Offset = i1(ignition::math::Pose3d(1, 0, 0, 0, 0, 0));
  EXPECT_TRUE(i1.MOI(
        ignition::math::Pose3d(1, 0, 0, 0, 0, 0)) == i1Offset.MOI());
  gzdbg << i1.Pose() << " : " <<  i1Offset.Pose() << "\n";
  EXPECT_TRUE(i1.Pose() == ignition::math::Pose3d(1, 0, 0, 0, 0, 0)+
      i1Offset.Pose());

  // Test MOI
  ignition::math::Matrix3d i11 =
    i1.MOI(ignition::math::Pose3d(1, 0, 0, 0, 0, 0));
  EXPECT_NEAR(i11(2, 2), 1.3, TOL);
  gzdbg << "i11:\n" << i11 << "\n";

  // Get i2 from origin of link
  physics::Inertial i3;
  i3.SetMOI(i2.MOI(ignition::math::Pose3d()));
  EXPECT_NEAR(i3.IXX(), 1 + 2, TOL);
  EXPECT_NEAR(i3.IYY(), 2 + 2, TOL);
  EXPECT_NEAR(i3.IZZ(), 3, TOL);

  gzdbg << "i2:\n" << i2 << "\n";
  gzdbg << "R:\n" << ignition::math::Matrix3d(
      ignition::math::Quaterniond(0, 0, 0.5*IGN_PI)) << "\n";
  gzdbg << "I:\n" << i2.MOI() << "\n";
  i2.SetMOI(i2.MOI(ignition::math::Pose3d(0, 0, 1, 0, 0, 0.5*IGN_PI)));
  gzdbg << "i2:\n" << i2 << "\n";
  EXPECT_NEAR(i2.IXX(), 2, TOL);
  EXPECT_NEAR(i2.IYY(), 1, TOL);
  EXPECT_NEAR(i2.IZZ(), 3, TOL);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

