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
#include "gazebo/physics/Joint.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define TOL 1e-6
#define TOL_CONT 2.0

using namespace gazebo;

class JointLimitTest : public ServerFixture,
                             public testing::WithParamInterface<const char*>
{
  /// \brief Load example world with a few joints
  /// Measure / verify joint limit enforcement
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void HingeJointLimit(const std::string &_physicsEngine);
  private: double last_error;
};

/////////////////////////////////////////////////
void JointLimitTest::HingeJointLimit(const std::string &_physicsEngine)
{
  // Load our force torque test world
  Load("worlds/joint_limit_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // quick basic sim check
  // sanity checks: get step size:
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";
  // sanity checks: simulate 1 step and see that sim time moved forward
  world->Step(1);
  double t = world->GetSimTime().Double();
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get joint and get force torque
  physics::ModelPtr model_1 = world->GetModel("model_1");
  ASSERT_TRUE(model_1 != NULL);
  physics::LinkPtr link_1 = model_1->GetLink("link_1");
  physics::LinkPtr link_2 = model_1->GetLink("link_2");
  physics::LinkPtr link_3 = model_1->GetLink("link_3");
  physics::LinkPtr link_4 = model_1->GetLink("link_4");
  physics::JointPtr joint_01 = model_1->GetJoint("joint_01");
  physics::JointPtr joint_12 = model_1->GetJoint("joint_12");
  physics::JointPtr joint_23 = model_1->GetJoint("joint_23");
  physics::JointPtr joint_34 = model_1->GetJoint("joint_34");
  ASSERT_TRUE(link_1 != NULL);
  ASSERT_TRUE(link_2 != NULL);
  ASSERT_TRUE(link_3 != NULL);
  ASSERT_TRUE(link_4 != NULL);
  ASSERT_TRUE(joint_01 != NULL);
  ASSERT_TRUE(joint_12 != NULL);
  ASSERT_TRUE(joint_23 != NULL);
  ASSERT_TRUE(joint_34 != NULL);

  gzdbg << "Test: drive joint_01 to positive limit with constant force.\n";
  gzdbg << "-----------------------------------------------------------\n";
  // getchar();
  if (_physicsEngine == "ode")
  {
    joint_01->SetParam("stop_erp", 0, 0.0);
    joint_01->SetParam("stop_cfm", 0, 10.0);
  }
  else if (_physicsEngine == "bullet")
  {
    gzerr << "Skipping test for now, need to fix gazebo bullet joint limit\n";
    gzerr << "TODO: duplicate the same stop_min_depth param.\n";
    return;
  }

  this->last_error = 1e16;
  for (unsigned int i = 0; i < 5000; ++i)
  {
    // What we are doing here:
    // drive joint_01 to it's limit, watch it's dynamic behavior.
    joint_01->SetForce(0, 100);
    world->Step(1);
    // see issue #1658, we should always do comparison of math::Angle values
    // by converting them to Radian() (double) first to prevent
    // inconsistency.
    if (joint_01->GetAngle(0).Radian() >= joint_01->GetUpperLimit(0).Radian())
    {
      double error = joint_01->GetAngle(0).Radian()
                   - joint_01->GetUpperLimit(0).Radian();
      // gzdbg << "t: [" << world->GetSimTime().Double()
      //       << "] pos: [" << joint_01->GetAngle(0)
      //       << "] >= lim: [" << joint_01->GetUpperLimit(0)
      //       << "] err: [" << error
      //       << "] >=: [" << (joint_01->GetAngle(0).Radian()
      //                      >= joint_01->GetUpperLimit(0).Radian())
      //       << "]\n";
      // getchar();
      ASSERT_TRUE(
        (joint_01->GetAngle(0)-joint_01->GetUpperLimit(0)).Radian() >= 0);

      // expect error to decrease, try to detect if new error is
      // 1 order of magnitude larger than last error
      // This can happen if joint limit is satisfied and removed for
      // one time step before being reactivated. One fix to this
      // is by using something similar to surface layer margin
      // for contacts here, see deps/opende/src/joints/joint.cpp
      // variable stop_min_depth. Without the margin, you'll
      // see the error drop down to 1e-16, then limit joint
      // gets removed for one time step, and following that the joint
      // limit violation can be arbitrarily large depending on force
      // applied.
      EXPECT_LT(error / this->last_error, 10.0);
      this->last_error = error;
    }
    else
    {
      // gzdbg << "\n\nLimit constraint removed\n\n";
      // no limit violation, preset error to something large
      // this->last_error = 1e16;
    }

  }
  // test negative torque and joint limit
  this->last_error = 1e16;
  for (unsigned int i = 0; i < 5000; ++i)
  {
    // What we are doing here:
    // drive joint_01 to it's limit, watch it's dynamic behavior.
    joint_01->SetForce(0, -100);
    world->Step(1);
    // see issue #1658, we should always do comparison of math::Angle values
    // by converting them to Radian() (double) first to prevent
    // inconsistency.
    if (joint_01->GetAngle(0).Radian() >= joint_01->GetUpperLimit(0).Radian())
    {
      double error = joint_01->GetAngle(0).Radian()
                   - joint_01->GetUpperLimit(0).Radian();
      // gzdbg << "t: [" << world->GetSimTime().Double()
      //       << "] pos: [" << joint_01->GetAngle(0)
      //       << "] >= lim: [" << joint_01->GetUpperLimit(0)
      //       << "] err: [" << error
      //       << "] >=: [" << (joint_01->GetAngle(0).Radian()
      //                      >= joint_01->GetUpperLimit(0).Radian())
      //       << "]\n";
      // getchar();
      ASSERT_TRUE(
        (joint_01->GetAngle(0)-joint_01->GetUpperLimit(0)).Radian() >= 0);

      // expect error to decrease, try to detect if new error is
      // 1 order of magnitude larger than last error
      // This can happen if joint limit is satisfied and removed for
      // one time step before being reactivated. One fix to this
      // is by using something similar to surface layer margin
      // for contacts here, see deps/opende/src/joints/joint.cpp
      // variable stop_min_depth. Without the margin, you'll
      // see the error drop down to 1e-16, then limit joint
      // gets removed for one time step, and following that the joint
      // limit violation can be arbitrarily large depending on force
      // applied.
      EXPECT_LT(error / this->last_error, 10.0);
      this->last_error = error;
    }
    else
    {
      // gzdbg << "\n\nLimit constraint removed\n\n";
      // no limit violation, preset error to something large
      // this->last_error = 1e16;
    }

  }
}

/////////////////////////////////////////////////
TEST_P(JointLimitTest, HingeJointLimit)
{
  HingeJointLimit(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointLimitTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
