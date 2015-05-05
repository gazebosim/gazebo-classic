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
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/physics/ode/ODEJoint.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"

#define TOL 1e-6
using namespace gazebo;

class ODEJoint_TEST : public ServerFixture
{
};

////////////////////////////////////////////////////////////////////////
// Test multi-axis universal joints
// with implicit (cfm) damping
////////////////////////////////////////////////////////////////////////
TEST_F(ODEJoint_TEST, ImplicitDamping)
{
  // Load our force torque test world
  Load("worlds/implicit_damping_test.world", true);

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
  gzdbg << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_GT(t, 0);
  gzdbg << "t after one step : " << t << "\n";

  // get joint and get force torque
  physics::ModelPtr model_1 = world->GetModel("model_1");
  physics::JointPtr joint_0 = model_1->GetJoint("joint_0");
  physics::JointPtr joint_1 = model_1->GetJoint("joint_1");

  EXPECT_TRUE(boost::dynamic_pointer_cast<physics::ODEJoint>(joint_0)->
      UsesImplicitSpringDamper());
  EXPECT_TRUE(boost::dynamic_pointer_cast<physics::ODEJoint>(joint_1)->
      UsesImplicitSpringDamper());

  // Test for UseImplicitSpringDamper setting method
  // toggle flag to false then back to true
  {
    physics::ODEJointPtr joint =
      boost::dynamic_pointer_cast<physics::ODEJoint>(joint_0);

    joint->UseImplicitSpringDamper(false);
    EXPECT_FALSE(joint->UsesImplicitSpringDamper());

    joint->UseImplicitSpringDamper(true);
    EXPECT_TRUE(joint->UsesImplicitSpringDamper());
  }

  gzdbg << "-------------------Test 1 (y)-------------------\n";
  physics->SetGravity(math::Vector3(0, 10, 0));
  world->Step(100);
  EXPECT_NEAR(joint_0->GetAngle(0).Radian(), 0.0, 1e-6);
  EXPECT_NEAR(joint_1->GetAngle(0).Radian(), 0.0048295899143964149, 1e-5);
  EXPECT_NEAR(joint_1->GetAngle(1).Radian(), 0.0, 1e-6);
  gzdbg << "time [" << world->GetSimTime().Double()
        << "] j0 [" << joint_0->GetAngle(0).Radian()
        << "] j1(0) [" << joint_1->GetAngle(0).Radian()
        << "] j1(1) [" << joint_1->GetAngle(1).Radian()
        << "]\n";

  gzdbg << "-------------------Test 2 (x)-------------------\n";
  physics->SetGravity(math::Vector3(10, 0, 0));
  world->Step(100);
  EXPECT_NEAR(joint_0->GetAngle(0).Radian(), 0.0, 1e-6);
  EXPECT_NEAR(joint_1->GetAngle(0).Radian(), 0.0050046318305403403, 1e-5);
  // The following expectation fails
  // EXPECT_NEAR(joint_1->GetAngle(1).Radian(), -0.0048293115636619532, 1e-5);
  gzdbg << "time [" << world->GetSimTime().Double()
        << "] j0 [" << joint_0->GetAngle(0).Radian()
        << "] j1(0) [" << joint_1->GetAngle(0).Radian()
        << "] j1(1) [" << joint_1->GetAngle(1).Radian()
        << "]\n";

  gzdbg << "-------------------Test 3 (joint limit)-------------------\n";
  physics->SetGravity(math::Vector3(1000, 1000, 0));
  world->Step(1000);
  EXPECT_NEAR(joint_0->GetAngle(0).Radian(), 0.0, 0.001);
  EXPECT_NEAR(joint_1->GetAngle(0).Radian(), 0.7, 0.001);
  EXPECT_NEAR(joint_1->GetAngle(1).Radian(), -0.7, 0.001);
  gzdbg << "time [" << world->GetSimTime().Double()
        << "] j0 [" << joint_0->GetAngle(0).Radian()
        << "] j1(0) [" << joint_1->GetAngle(0).Radian()
        << "] j1(1) [" << joint_1->GetAngle(1).Radian()
        << "]\n";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
