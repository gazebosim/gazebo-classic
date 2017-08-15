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
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), "ode");

  physics->SetGravity(ignition::math::Vector3d(0, 0, -50));

  // simulate 1 step
  world->Step(1);
  double t = world->SimTime().Double();

  // get time step size
  double dt = world->Physics()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzdbg << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_GT(t, 0);
  gzdbg << "t after one step : " << t << "\n";

  // get joint and get force torque
  physics::ModelPtr model_1 = world->ModelByName("model_1");
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
  physics->SetGravity(ignition::math::Vector3d(0, 10, 0));
  world->Step(100);
  EXPECT_NEAR(joint_0->Position(0), 0.0, 1e-6);
  EXPECT_NEAR(joint_1->Position(0), 0.0048295899143964149, 1e-5);
  EXPECT_NEAR(joint_1->Position(1), 0.0, 1e-6);
  gzdbg << "time [" << world->SimTime().Double()
        << "] j0 [" << joint_0->Position(0)
        << "] j1(0) [" << joint_1->Position(0)
        << "] j1(1) [" << joint_1->Position(1)
        << "]\n";

  gzdbg << "-------------------Test 2 (x)-------------------\n";
  physics->SetGravity(ignition::math::Vector3d(10, 0, 0));
  world->Step(100);
  EXPECT_NEAR(joint_0->Position(0), 0.0, 1e-6);
  EXPECT_NEAR(joint_1->Position(0), 0.0050046318305403403, 1e-5);
  // The following expectation fails
  // EXPECT_NEAR(joint_1->Position(1), -0.0048293115636619532, 1e-5);
  gzdbg << "time [" << world->SimTime().Double()
        << "] j0 [" << joint_0->Position(0)
        << "] j1(0) [" << joint_1->Position(0)
        << "] j1(1) [" << joint_1->Position(1)
        << "]\n";

  gzdbg << "-------------------Test 3 (joint limit)-------------------\n";
  physics->SetGravity(ignition::math::Vector3d(1000, 1000, 0));
  world->Step(1000);
  EXPECT_NEAR(joint_0->Position(0), 0.0, 0.001);
  EXPECT_NEAR(joint_1->Position(0), 0.7, 0.001);
  EXPECT_NEAR(joint_1->Position(1), -0.7, 0.001);
  gzdbg << "time [" << world->SimTime().Double()
        << "] j0 [" << joint_0->Position(0)
        << "] j1(0) [" << joint_1->Position(0)
        << "] j1(1) [" << joint_1->Position(1)
        << "]\n";
}

////////////////////////////////////////////////////////////////////////
// Test joint limits after moving link center of gravity
TEST_F(ODEJoint_TEST, JointLimitsMoveCoG)
{
  // Load our force torque test world
  Load("test/worlds/joint_limits_test.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);
  EXPECT_EQ(world->Gravity(), ignition::math::Vector3d(0, 0, -9.8));

  // Verify physics
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), "ode");

  physics::ModelPtr model= world->ModelByName("model");
  ASSERT_TRUE(model != nullptr);
  physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != nullptr);
  physics::JointPtr joint = model->GetJoint("joint");
  ASSERT_TRUE(joint != nullptr);

  // Initial link pose
  EXPECT_EQ(link->WorldPose(),
      ignition::math::Pose3d(0.0, 0.5, 1.5, 0.0, 0.0, 0.0));
  EXPECT_NEAR(joint->Position(0), 0.0, 1e-3);

  // Let the link rotate until the joint hits its limit
  world->Step(1000);
  EXPECT_EQ(link->WorldPose(),
      ignition::math::Pose3d(0, 0.5*std::sqrt(2.0), 1, -0.785398, 0, 0));
  EXPECT_NEAR(joint->Position(0), -0.785398, 1e-3);

  // Update the link's center of mgravity
  physics::InertialPtr inertial = link->GetInertial();
  ignition::math::Pose3d cogPose = inertial->Pose();
  cogPose.Pos().Y() += 1.0;
  inertial->SetCoG(cogPose);
  link->UpdateMass();

  // Check that the link's pose is unchanged
  world->Step(1000);
  EXPECT_EQ(link->WorldPose(),
      ignition::math::Pose3d(0, 0.5*std::sqrt(2.0), 1, -0.785398, 0, 0));
  EXPECT_NEAR(joint->Position(0), -0.785398, 1e-3);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
