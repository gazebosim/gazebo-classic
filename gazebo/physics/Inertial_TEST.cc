/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/Inertial.hh"

#define TOL 1e-6
using namespace gazebo;

class Inertial_TEST : public ServerFixture
{
};

////////////////////////////////////////////////////////////////////////
// Test world template
////////////////////////////////////////////////////////////////////////
TEST_F(Inertial_TEST, InertialWorld)
{
  // Load our inertial test world
  Load("worlds/inertial_test.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), "ode");

  physics->SetGravity(math::Vector3(0, 0, -50));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // verify that time moves forward
  EXPECT_NEAR(t, dt, TOL);

  Unload();
}

////////////////////////////////////////////////////////////////////////
// Test basic Inertial functions
////////////////////////////////////////////////////////////////////////
TEST_F(Inertial_TEST, InertialOperators)
{
  physics::Inertial i1;
  physics::Inertial i2;
  i1.SetMass(1.0);
  i2.SetMass(2.0);
  i1.SetCoG(math::Pose(0, 0, 0, 0, 0, 0));
  i2.SetCoG(math::Pose(0, 0, 1, 0, 0, 0));

  i1.SetIXX(0.1);
  i1.SetIYY(0.2);
  i1.SetIZZ(0.3);
  i1.SetIXY(0.0);
  i1.SetIXZ(0.0);
  i1.SetIYZ(0.0);

  i2.SetMOI(math::Matrix3(1, 0, 0,
                          0, 2, 0,
                          0, 0, 3));

  // Test addition
  physics::Inertial isum = i1 + i2;
  gzdbg << "isum: \n"
        << isum << "\n";
  gzdbg << "i1 new cg: \n"
        << i1.GetMOI(isum.GetPose()) << "\n";
  gzdbg << "i2 new cg: \n"
        << i2.GetMOI(isum.GetPose()) << "\n";
  EXPECT_NEAR(isum.GetPose().pos.z, 2.0/3.0, TOL);
  EXPECT_NEAR(isum.GetIXX(),
    1.0 + 0.1 + 1.0*(2.0/3.0)*(2.0/3.0)
              + 2.0*(1-2.0/3.0)*(1-2.0/3.0), TOL);
  EXPECT_NEAR(isum.GetIYY(),
    2.0 + 0.2 + 1.0*(2.0/3.0)*(2.0/3.0)
              + 2.0*(1-2.0/3.0)*(1-2.0/3.0), TOL);
  EXPECT_NEAR(isum.GetIZZ(), 3.0 + 0.3, TOL);

  // Test GetInertial(offset)
  physics::Inertial i1Offset = i1.GetInertial(math::Pose(0, 0, 0, 0, 0, 0));
  EXPECT_TRUE(i1.GetMOI() == i1Offset.GetMOI());
  EXPECT_TRUE(i1.GetPose() == i1Offset.GetPose());
  i1Offset = i1.GetInertial(math::Pose(1, 0, 0, 0, 0, 0));
  EXPECT_TRUE(i1.GetMOI(math::Pose(1, 0, 0, 0, 0, 0)) == i1Offset.GetMOI());
  gzdbg << i1.GetPose() << " : " <<  i1Offset.GetPose() << "\n";
  EXPECT_TRUE(i1.GetPose() == math::Pose(1, 0, 0, 0, 0, 0)+i1Offset.GetPose());

  // Test GetMOI
  math::Matrix3 i11 = i1.GetMOI(math::Pose(1, 0, 0, 0, 0, 0));
  EXPECT_NEAR(i11[2][2], 1.3, TOL);
  gzdbg << "i11:\n" << i11 << "\n";

  // Get i2 from origin of link
  physics::Inertial i3;
  i3.SetMOI(i2.GetMOI(math::Pose()));
  EXPECT_NEAR(i3.GetIXX(), 1 + 2, TOL);
  EXPECT_NEAR(i3.GetIYY(), 2 + 2, TOL);
  EXPECT_NEAR(i3.GetIZZ(), 3, TOL);

  gzdbg << "i2:\n" << i2 << "\n";
  gzdbg << "R:\n" << math::Quaternion(0, 0, 0.5*M_PI).GetAsMatrix3() << "\n";
  gzdbg << "I:\n" << i2.GetMOI() << "\n";
  i2.SetMOI(i2.GetMOI(math::Pose(0, 0, 1, 0, 0, 0.5*M_PI)));
  gzdbg << "i2:\n" << i2 << "\n";
  EXPECT_NEAR(i2.GetIXX(), 2, TOL);
  EXPECT_NEAR(i2.GetIYY(), 1, TOL);
  EXPECT_NEAR(i2.GetIZZ(), 3, TOL);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

