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
#include "gazebo/physics/physics.hh"
#include "test/ServerFixture.hh"

#define TOL 1e-6
using namespace gazebo;

class ODEGearboxJoint_TEST : public ServerFixture
{
  public: void GearboxTest(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// GearboxTest:
// start gearbox.world, apply balancing forces across geared members,
// check for equilibrium.
////////////////////////////////////////////////////////////////////////
void ODEGearboxJoint_TEST::GearboxTest(const std::string &_physicsEngine)
{
  // load an empty world
  Load("worlds/gearbox.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();

  // Assume gravity vector points down z axis only.
  EXPECT_NEAR(g.x, 0, 1e-6);
  EXPECT_NEAR(g.y, 0, 1e-6);
  EXPECT_LE(g.z, 0);

  physics::ModelPtr model = world->GetModel("model_1");
  physics::JointPtr joint1 = model->GetJoint("joint_12");
  physics::JointPtr joint3 = model->GetJoint("joint_23");
  int steps = 10000;
  for (int i = 0; i < steps; ++i)
  {
    joint1->SetForce(0, -1.5);
    joint3->SetForce(0,  1.0);
    world->StepWorld(1);
    if (i%1000 == 0)
      gzdbg << "gearbox time [" << world->GetSimTime().Double()
            << "] vel [" << joint1->GetVelocity(0)
            << "] pose [" << joint1->GetAngle(0).Radian()
            << "] vel [" << joint3->GetVelocity(0)
            << "] pose [" << joint3->GetAngle(0).Radian()
            << "]\n";
  }
  EXPECT_NEAR(joint1->GetVelocity(0), 0, 1e-6);
  EXPECT_NEAR(joint1->GetVelocity(0), 0, 1e-6);
  EXPECT_NEAR(joint3->GetAngle(0).Radian(), 0, 1e-6);
  EXPECT_NEAR(joint3->GetAngle(0).Radian(), 0, 1e-6);

  // slight imbalance
  for (int i = 0; i < steps; ++i)
  {
    joint1->SetForce(0, -1.0);
    joint3->SetForce(0,  1.0);
    world->StepWorld(1);
    if (i%1000 == 0)
      gzdbg << "gearbox time [" << world->GetSimTime().Double()
            << "] vel [" << joint1->GetVelocity(0)
            << "] pose [" << joint1->GetAngle(0).Radian()
            << "] vel [" << joint3->GetVelocity(0)
            << "] pose [" << joint3->GetAngle(0).Radian()
            << "]\n";
  }
  EXPECT_GT(joint1->GetVelocity(0), 0);
  EXPECT_GT(joint1->GetVelocity(0), 0);
  EXPECT_GT(joint3->GetAngle(0).Radian(), 0);
  EXPECT_GT(joint3->GetAngle(0).Radian(), 0);
}

TEST_F(ODEGearboxJoint_TEST, GearboxTestODE)
{
  GearboxTest("ode");
}

// #ifdef HAVE_BULLET
// TEST_F(ODEGearboxJoint_TEST, GearboxTestBullet)
// {
//   GearboxTest("bullet");
// }
// #endif  // HAVE_BULLET


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
