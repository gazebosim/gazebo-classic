/*
 * Copyright 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/physics/ode/ODEGearboxJoint.hh"
#include "test/ServerFixture.hh"

#define TOL 1e-6
using namespace gazebo;

class ODEGearboxJoint_TEST : public ServerFixture
{
  public: void GearboxTest(const std::string &_physicsEngine);
  public: void SetGearRatio(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// GearboxTest:
// start gearbox.world, apply balancing forces across geared members,
// check for equilibrium.
////////////////////////////////////////////////////////////////////////
void ODEGearboxJoint_TEST::GearboxTest(const std::string &_physicsEngine)
{
  // load gearbox world
  Load("worlds/gearbox.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->GetModel("model_1");
  physics::JointPtr joint1 = model->GetJoint("joint_12");
  physics::JointPtr joint3 = model->GetJoint("joint_23");

  physics::JointPtr gearboxPtr = model->GetJoint("joint_13");
  ASSERT_TRUE(gearboxPtr != NULL);
  ASSERT_TRUE(gearboxPtr->HasType(physics::Base::GEARBOX_JOINT));
  boost::shared_ptr<physics::ODEGearboxJoint> gearbox =
    boost::dynamic_pointer_cast<physics::ODEGearboxJoint>(gearboxPtr);
  double gearRatio = gearbox->GetGearRatio();
  EXPECT_NEAR(gearRatio, -1.5, TOL);
  double force3 = 1.0;
  double force1 = force3 * gearRatio;

  int steps = 10000;
  for (int i = 0; i < steps; ++i)
  {
    joint1->SetForce(0, force1);
    joint3->SetForce(0, force3);
    world->Step(1);
    if (i%1000 == 0)
      gzdbg << "gearbox time [" << world->GetSimTime().Double()
            << "] vel [" << joint1->GetVelocity(0)
            << "] pose [" << joint1->GetAngle(0).Radian()
            << "] vel [" << joint3->GetVelocity(0)
            << "] pose [" << joint3->GetAngle(0).Radian()
            << "]\n";
    EXPECT_NEAR(joint1->GetVelocity(0), 0, 1e-6);
    EXPECT_NEAR(joint3->GetVelocity(0), 0, 1e-6);
    EXPECT_NEAR(joint1->GetAngle(0).Radian(), 0, 1e-6);
    EXPECT_NEAR(joint3->GetAngle(0).Radian(), 0, 1e-6);
  }

  // slight imbalance
  for (int i = 0; i < steps; ++i)
  {
    joint1->SetForce(0, -force3);
    joint3->SetForce(0,  force3);
    world->Step(1);
    if (i%1000 == 0)
      gzdbg << "gearbox time [" << world->GetSimTime().Double()
            << "] vel [" << joint1->GetVelocity(0)
            << "] pose [" << joint1->GetAngle(0).Radian()
            << "] vel [" << joint3->GetVelocity(0)
            << "] pose [" << joint3->GetAngle(0).Radian()
            << "]\n";
    EXPECT_GT(joint1->GetVelocity(0), 0);
    EXPECT_GT(joint3->GetVelocity(0), 0);
    EXPECT_GT(joint1->GetAngle(0).Radian(), 0);
    EXPECT_GT(joint3->GetAngle(0).Radian(), 0);
    EXPECT_NEAR(joint1->GetVelocity(0)*gearRatio, -joint3->GetVelocity(0), TOL);
    EXPECT_NEAR(joint1->GetAngle(0).Radian()*gearRatio,
               -joint3->GetAngle(0).Radian(), TOL);
  }
}

TEST_F(ODEGearboxJoint_TEST, GearboxTestODE)
{
  GearboxTest("ode");
}

////////////////////////////////////////////////////////////////////////
// SetGearRatio:
// start gearbox.world, set a new gear ratio,
// apply balancing forces across geared members, and check for equilibrium.
////////////////////////////////////////////////////////////////////////
void ODEGearboxJoint_TEST::SetGearRatio(const std::string &_physicsEngine)
{
  // load gearbox world
  Load("worlds/gearbox.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->GetModel("model_1");
  physics::JointPtr joint1 = model->GetJoint("joint_12");
  physics::JointPtr joint3 = model->GetJoint("joint_23");

  physics::JointPtr gearboxPtr = model->GetJoint("joint_13");
  ASSERT_TRUE(gearboxPtr != NULL);
  ASSERT_TRUE(gearboxPtr->HasType(physics::Base::GEARBOX_JOINT));
  boost::shared_ptr<physics::ODEGearboxJoint> gearbox =
    boost::dynamic_pointer_cast<physics::ODEGearboxJoint>(gearboxPtr);
  double gearRatio = -2.5;
  gearbox->SetGearRatio(gearRatio);
  EXPECT_NEAR(gearRatio, gearbox->GetGearRatio(), TOL);
  double force3 = 1.0;
  double force1 = force3 * gearRatio;

  int steps = 10000;
  for (int i = 0; i < steps; ++i)
  {
    joint1->SetForce(0, force1);
    joint3->SetForce(0, force3);
    world->Step(1);
    if (i%1000 == 0)
      gzdbg << "gearbox time [" << world->GetSimTime().Double()
            << "] vel [" << joint1->GetVelocity(0)
            << "] pose [" << joint1->GetAngle(0).Radian()
            << "] vel [" << joint3->GetVelocity(0)
            << "] pose [" << joint3->GetAngle(0).Radian()
            << "]\n";
    EXPECT_NEAR(joint1->GetVelocity(0), 0, 1e-6);
    EXPECT_NEAR(joint3->GetVelocity(0), 0, 1e-6);
    EXPECT_NEAR(joint1->GetAngle(0).Radian(), 0, 1e-6);
    EXPECT_NEAR(joint3->GetAngle(0).Radian(), 0, 1e-6);
  }

  // slight imbalance
  for (int i = 0; i < steps; ++i)
  {
    joint1->SetForce(0, -force3);
    joint3->SetForce(0,  force3);
    world->Step(1);
    if (i%1000 == 0)
      gzdbg << "gearbox time [" << world->GetSimTime().Double()
            << "] vel [" << joint1->GetVelocity(0)
            << "] pose [" << joint1->GetAngle(0).Radian()
            << "] vel [" << joint3->GetVelocity(0)
            << "] pose [" << joint3->GetAngle(0).Radian()
            << "]\n";
    EXPECT_GT(joint1->GetVelocity(0), 0);
    EXPECT_GT(joint3->GetVelocity(0), 0);
    EXPECT_GT(joint1->GetAngle(0).Radian(), 0);
    EXPECT_GT(joint3->GetAngle(0).Radian(), 0);
    EXPECT_NEAR(joint1->GetVelocity(0)*gearRatio, -joint3->GetVelocity(0), TOL);
    EXPECT_NEAR(joint1->GetAngle(0).Radian()*gearRatio,
               -joint3->GetAngle(0).Radian(), TOL);
  }
}

TEST_F(ODEGearboxJoint_TEST, SetGearRatioODE)
{
  SetGearRatio("ode");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
