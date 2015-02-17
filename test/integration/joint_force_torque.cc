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
#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"

#define TOL 1e-6
#define TOL_CONT 2.0

using namespace gazebo;

class JointForceTorqueTest : public ServerFixture,
                             public testing::WithParamInterface<const char*>
{
  /// \brief Load example world with a few joints
  /// Measure / verify static force torques against analytical answers.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void ForceTorque1(const std::string &_physicsEngine);

  /// \brief Load example world with a few joints
  /// Measure / verify static force torques against analytical answers.
  /// Change gravity to tip over the joints.
  /// Wait until joint stops are hit and joint motion settles,
  /// then check force torques values against analytical values.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void ForceTorque2(const std::string &_physicsEngine);

  /// \brief Load example world with a few joints.
  /// Servo the joints to a fixed target position using simple PID controller.
  /// Measure / verify static force torques against analytical answers.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetForceTorqueWithAppliedForce(
    const std::string &_physicsEngine);

  /// \brief Create a hinge joint between link and world.
  /// Apply force and check acceleration against analytical solution.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void JointTorqueTest(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void JointForceTorqueTest::ForceTorque1(const std::string &_physicsEngine)
{
  // Load our force torque test world
  Load("worlds/force_torque_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(math::Vector3(0, 0, -50));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get joint and get force torque
  physics::ModelPtr model_1 = world->GetModel("model_1");
  physics::LinkPtr link_1 = model_1->GetLink("link_1");
  physics::LinkPtr link_2 = model_1->GetLink("link_2");
  physics::JointPtr joint_01 = model_1->GetJoint("joint_01");
  physics::JointPtr joint_12 = model_1->GetJoint("joint_12");

  gzlog << "-------------------Test 1-------------------\n";
  for (unsigned int i = 0; i < 10; ++i)
  {
    world->Step(1);
    // test joint_01 wrench
    physics::JointWrench wrench_01 = joint_01->GetForceTorque(0u);
    EXPECT_DOUBLE_EQ(wrench_01.body1Force.x,    0.0);
    EXPECT_DOUBLE_EQ(wrench_01.body1Force.y,    0.0);
    EXPECT_DOUBLE_EQ(wrench_01.body1Force.z, 1000.0);
    EXPECT_DOUBLE_EQ(wrench_01.body1Torque.x,   0.0);
    EXPECT_DOUBLE_EQ(wrench_01.body1Torque.y,   0.0);
    EXPECT_DOUBLE_EQ(wrench_01.body1Torque.z,   0.0);

    EXPECT_DOUBLE_EQ(wrench_01.body2Force.x,  -wrench_01.body1Force.x);
    EXPECT_DOUBLE_EQ(wrench_01.body2Force.y,  -wrench_01.body1Force.y);
    EXPECT_DOUBLE_EQ(wrench_01.body2Force.z,  -wrench_01.body1Force.z);
    EXPECT_DOUBLE_EQ(wrench_01.body2Torque.x, -wrench_01.body1Torque.x);
    EXPECT_DOUBLE_EQ(wrench_01.body2Torque.y, -wrench_01.body1Torque.y);
    EXPECT_DOUBLE_EQ(wrench_01.body2Torque.z, -wrench_01.body1Torque.z);

    gzlog << "link_1 pose [" << link_1->GetWorldPose()
          << "] velocity [" << link_1->GetWorldLinearVel()
          << "]\n";
    gzlog << "link_2 pose [" << link_2->GetWorldPose()
          << "] velocity [" << link_2->GetWorldLinearVel()
          << "]\n";
    gzlog << "joint_01 force torque : "
          << "force1 [" << wrench_01.body1Force
          << " / 0 0 1000"
          << "] torque1 [" << wrench_01.body1Torque
          << " / 0 0 0"
          << "] force2 [" << wrench_01.body2Force
          << " / 0 0 -1000"
          << "] torque2 [" << wrench_01.body2Torque
          << " / 0 0 0"
          << "]\n";

    // test joint_12 wrench
    physics::JointWrench wrench_12 = joint_12->GetForceTorque(0u);
    EXPECT_DOUBLE_EQ(wrench_12.body1Force.x,    0.0);
    EXPECT_DOUBLE_EQ(wrench_12.body1Force.y,    0.0);
    EXPECT_DOUBLE_EQ(wrench_12.body1Force.z,  500.0);
    EXPECT_DOUBLE_EQ(wrench_12.body1Torque.x,   0.0);
    EXPECT_DOUBLE_EQ(wrench_12.body1Torque.y,   0.0);
    EXPECT_DOUBLE_EQ(wrench_12.body1Torque.z,   0.0);

    EXPECT_DOUBLE_EQ(wrench_12.body2Force.x,  -wrench_12.body1Force.x);
    EXPECT_DOUBLE_EQ(wrench_12.body2Force.y,  -wrench_12.body1Force.y);
    EXPECT_DOUBLE_EQ(wrench_12.body2Force.z,  -wrench_12.body1Force.z);
    EXPECT_DOUBLE_EQ(wrench_12.body2Torque.x, -wrench_12.body1Torque.x);
    EXPECT_DOUBLE_EQ(wrench_12.body2Torque.y, -wrench_12.body1Torque.y);
    EXPECT_DOUBLE_EQ(wrench_12.body2Torque.z, -wrench_12.body1Torque.z);

    gzlog << "link_1 pose [" << link_1->GetWorldPose()
          << "] velocity [" << link_1->GetWorldLinearVel()
          << "]\n";
    gzlog << "link_2 pose [" << link_2->GetWorldPose()
          << "] velocity [" << link_2->GetWorldLinearVel()
          << "]\n";
    gzlog << "joint_12 force torque : "
          << "force1 [" << wrench_12.body1Force
          << " / 0 0 500"
          << "] torque1 [" << wrench_12.body1Torque
          << " / 0 0 0"
          << "] force2 [" << wrench_12.body2Force
          << " / 0 0 -500"
          << "] torque2 [" << wrench_12.body2Torque
          << " / 0 0 0"
          << "]\n";
  }
}

/////////////////////////////////////////////////
void JointForceTorqueTest::ForceTorque2(const std::string &_physicsEngine)
{
  // Load our force torque test world
  Load("worlds/force_torque_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(math::Vector3(0, 0, -50));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get joint and get force torque
  physics::ModelPtr model_1 = world->GetModel("model_1");
  physics::LinkPtr link_1 = model_1->GetLink("link_1");
  physics::LinkPtr link_2 = model_1->GetLink("link_2");
  physics::JointPtr joint_01 = model_1->GetJoint("joint_01");
  physics::JointPtr joint_12 = model_1->GetJoint("joint_12");

  // perturbe joints so top link topples over, then remeasure
  physics->SetGravity(math::Vector3(-30, 10, -50));
  // tune joint stop properties
  joint_01->SetParam("stop_erp", 0, 0.02);
  joint_12->SetParam("stop_erp", 0, 0.02);
  // wait for dynamics to stabilize
  world->Step(2000);
  // check force torques in new system
  gzlog << "\n-------------------Test 2-------------------\n";
  for (unsigned int i = 0; i < 5; ++i)
  {
    world->Step(1);
    // Dbg joint_01 force torque :
    //   force1 [600 -200 999.99999600000001 / 600 -1000 -200]
    //   torque1 [749.999819 82.840868 -450.00009699999998 / 750 450 0]
    //   force2 [-600 999.99976200000003 200.00117299999999 / -600 1000 200]
    //   torque2 [-749.999819 -450 -82.841396000000003 / -750 -450 0]
    // Dbg joint_12 force torque :
    //   force1 [300 -499.99987900000002 -100.000587 / 300 -500 -100]
    //   torque1 [249.99994000000001 150 82.841396000000003 / 250 150 0]
    //   force2 [-300.000407 499.99963500000001 100.000587 / -300 500 100]
    //   torque2 [-249.999818 -150.000203 -82.841396000000003 / -250 -150 0]

    // test joint_01 wrench
    physics::JointWrench wrench_01 = joint_01->GetForceTorque(0u);
    EXPECT_NEAR(wrench_01.body1Force.x,   600.0,  6.0);
    EXPECT_NEAR(wrench_01.body1Force.y,  -200.0, 10.0);
    EXPECT_NEAR(wrench_01.body1Force.z,  1000.0,  2.0);
    EXPECT_NEAR(wrench_01.body1Torque.x,  750.0,  7.5);
    EXPECT_NEAR(wrench_01.body1Torque.y,    0.0,  4.5);
    EXPECT_NEAR(wrench_01.body1Torque.z, -450.0,  0.1);

    EXPECT_NEAR(wrench_01.body2Force.x,  -600.0,  6.0);
    EXPECT_NEAR(wrench_01.body2Force.y,  1000.0, 10.0);
    if (_physicsEngine == "dart")
    {
      // DART needs greater tolerance due to joint limit violation
      // Please see issue #902
      EXPECT_NEAR(wrench_01.body2Force.z,   200.0,  8.6);
    }
    else
    {
      EXPECT_NEAR(wrench_01.body2Force.z,   200.0,  2.0);
    }
    EXPECT_NEAR(wrench_01.body2Torque.x, -750.0,  7.5);
    EXPECT_NEAR(wrench_01.body2Torque.y, -450.0,  4.5);
    EXPECT_NEAR(wrench_01.body2Torque.z,    0.0,  0.1);

    gzlog << "joint_01 force torque : "
          << "force1 [" << wrench_01.body1Force
          << " / 600 -200 1000"
          << "] torque1 [" << wrench_01.body1Torque
          << " / 750 0 450"
          << "] force2 [" << wrench_01.body2Force
          << " / -600 1000 200"
          << "] torque2 [" << wrench_01.body2Torque
          << " / -750 -450 0"
          << "]\n";

    gzlog << "joint angle1[" << std::setprecision(17) << joint_01->GetAngle(0)
          << "] angle2[" << joint_12->GetAngle(0) << "]\n";

    // test joint_12 wrench
    physics::JointWrench wrench_12 = joint_12->GetForceTorque(0u);
    EXPECT_NEAR(wrench_12.body1Force.x,   300.0,  3.0);
    EXPECT_NEAR(wrench_12.body1Force.y,  -500.0,  5.0);
    if (_physicsEngine == "dart")
    {
      // DART needs greater tolerance due to joint limit violation
      // Please see issue #902
      EXPECT_NEAR(wrench_12.body1Force.z,  -100.0,  4.3);
    }
    else
    {
      EXPECT_NEAR(wrench_12.body1Force.z,  -100.0,  1.0);
    }
    EXPECT_NEAR(wrench_12.body1Torque.x,  250.0,  5.0);
    EXPECT_NEAR(wrench_12.body1Torque.y,  150.0,  3.0);
    EXPECT_NEAR(wrench_12.body1Torque.z,    0.0,  0.1);

    // A good check is that
    // the computed body1Torque shoud in fact be opposite of body1Torque
    EXPECT_NEAR(wrench_12.body2Force.x,  -wrench_12.body1Force.x,  1e-1);
    EXPECT_NEAR(wrench_12.body2Force.y,  -wrench_12.body1Force.y,  1e-1);
    EXPECT_NEAR(wrench_12.body2Force.z,  -wrench_12.body1Force.z,  1e-1);
    EXPECT_NEAR(wrench_12.body2Torque.x, -wrench_12.body1Torque.x, 1e-1);
    EXPECT_NEAR(wrench_12.body2Torque.y, -wrench_12.body1Torque.y, 1e-1);
    EXPECT_NEAR(wrench_12.body2Torque.z, -wrench_12.body1Torque.z, 1e-1);

    gzlog << "joint_12 force torque : "
          << "force1 [" << wrench_12.body1Force
          << " / 300 -500 -100"
          << "] torque1 [" << wrench_12.body1Torque
          << " / 250 150 0"
          << "] force2 [" << wrench_12.body2Force
          << " / -300 500 100"
          << "] torque2 [" << wrench_12.body2Torque
          << " / -250 -150 0"
          << "]\n";
  }

  // simulate a few steps
  int steps = 20;
  world->Step(steps);
  t = world->GetSimTime().Double();
  EXPECT_GT(t, 0.99*dt*static_cast<double>(steps+1));
  gzdbg << "t after 20 steps : " << t << "\n";
}

/////////////////////////////////////////////////
void JointForceTorqueTest::GetForceTorqueWithAppliedForce(
  const std::string &_physicsEngine)
{
  // Explicit joint damping in bullet is causing this test to fail.
  if (_physicsEngine == "bullet")
  {
    gzerr << "Aborting test for bullet, see issue #619.\n";
    return;
  }

  // Load our force torque test world
  Load("worlds/force_torque_test2.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(math::Vector3(0, 0, -50));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_GT(t, 0);
  gzlog << "t after one step : " << t << "\n";

  // get joint and get force torque
  physics::ModelPtr model_1 = world->GetModel("boxes");
  physics::JointPtr joint_01 = model_1->GetJoint("joint1");
  physics::JointPtr joint_12 = model_1->GetJoint("joint2");

  gzlog << "------------------- PD CONTROL -------------------\n";
  static const double kp1 = 50000.0;
  static const double kp2 = 10000.0;
  static const double target1 = 0.0;
  static const double target2 = -0.25*M_PI;
  for (unsigned int i = 0; i < 3388; ++i)
  {
    // pd control
    double j1State = joint_01->GetAngle(0u).Radian();
    double j2State = joint_12->GetAngle(0u).Radian();
    double p1Error = target1 - j1State;
    double p2Error = target2 - j2State;
    double effort1 = kp1 * p1Error;
    double effort2 = kp2 * p2Error;
    joint_01->SetForce(0u, effort1);
    joint_12->SetForce(0u, effort2);

    world->Step(1);
    // test joint_01 wrench
    physics::JointWrench wrench_01 = joint_01->GetForceTorque(0u);

    if (i == 3387)
    {
      EXPECT_NEAR(wrench_01.body1Force.x,     0.0, TOL_CONT);
      EXPECT_NEAR(wrench_01.body1Force.y,     0.0, TOL_CONT);
      EXPECT_NEAR(wrench_01.body1Force.z,   300.0, TOL_CONT);
      EXPECT_NEAR(wrench_01.body1Torque.x,   25.0, TOL_CONT);
      EXPECT_NEAR(wrench_01.body1Torque.y, -175.0, TOL_CONT);
      EXPECT_NEAR(wrench_01.body1Torque.z,    0.0, TOL_CONT);

      EXPECT_NEAR(wrench_01.body2Force.x,  -wrench_01.body1Force.x,  TOL_CONT);
      EXPECT_NEAR(wrench_01.body2Force.y,  -wrench_01.body1Force.y,  TOL_CONT);
      EXPECT_NEAR(wrench_01.body2Force.z,  -wrench_01.body1Force.z,  TOL_CONT);
      EXPECT_NEAR(wrench_01.body2Torque.x, -wrench_01.body1Torque.x, TOL_CONT);
      EXPECT_NEAR(wrench_01.body2Torque.y, -wrench_01.body1Torque.y, TOL_CONT);
      EXPECT_NEAR(wrench_01.body2Torque.z, -wrench_01.body1Torque.z, TOL_CONT);

      gzlog << "joint_01 force torque : "
            << "step [" << i
            << "] GetForce [" << joint_01->GetForce(0u)
            << "] command [" << effort1
            << "] force1 [" << wrench_01.body1Force
            << "] torque1 [" << wrench_01.body1Torque
            << "] force2 [" << wrench_01.body2Force
            << "] torque2 [" << wrench_01.body2Torque
            << "]\n";
    }

    // test joint_12 wrench
    physics::JointWrench wrench_12 = joint_12->GetForceTorque(0u);
    if (i == 3387)
    {
      EXPECT_NEAR(wrench_12.body1Force.x,     0.0, TOL_CONT);
      EXPECT_NEAR(wrench_12.body1Force.y,     0.0, TOL_CONT);
      EXPECT_NEAR(wrench_12.body1Force.z,    50.0, TOL_CONT);
      EXPECT_NEAR(wrench_12.body1Torque.x,   25.0, TOL_CONT);
      EXPECT_NEAR(wrench_12.body1Torque.y,    0.0, TOL_CONT);
      EXPECT_NEAR(wrench_12.body1Torque.z,    0.0, TOL_CONT);

      EXPECT_NEAR(wrench_12.body2Force.x,   -35.355, TOL_CONT);
      EXPECT_NEAR(wrench_12.body2Force.y,     0.000, TOL_CONT);
      EXPECT_NEAR(wrench_12.body2Force.z,   -35.355, TOL_CONT);
      EXPECT_NEAR(wrench_12.body2Torque.x,  -17.678, TOL_CONT);
      EXPECT_NEAR(wrench_12.body2Torque.y,    0.000, TOL_CONT);
      EXPECT_NEAR(wrench_12.body2Torque.z,   17.678, TOL_CONT);

      gzlog << "joint_12 force torque : "
            << "step [" << i
            << "] GetForce [" << joint_12->GetForce(0u)
            << "] command [" << effort2
            << "] force1 [" << wrench_12.body1Force
            << "] torque1 [" << wrench_12.body1Torque
            << "] force2 [" << wrench_12.body2Force
            << "] torque2 [" << wrench_12.body2Torque
            << "]\n";
    }
    gzlog << "angles[" << i << "] 1[" << joint_01->GetAngle(0)
          << "] 2[" << joint_12->GetAngle(0)
          << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////
// Create a joint between link and world
// Apply force and check acceleration for correctness
////////////////////////////////////////////////////////////////////////
void JointForceTorqueTest::JointTorqueTest(const std::string &_physicsEngine)
{
  // Load our inertial test world
  Load("worlds/joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  {
    // get model
    physics::ModelPtr model = world->GetModel("model_1");
    ASSERT_TRUE(model != NULL);
    physics::LinkPtr link = model->GetLink("link_1");
    ASSERT_TRUE(link != NULL);
    physics::JointPtr joint = model->GetJoint("joint_01");

    double lastV = 0;
    double dt = world->GetPhysicsEngine()->GetMaxStepSize();
    for (unsigned int i = 0; i < 10; ++i)
    {
      double torque = 1.3;
      joint->SetForce(0, torque);
      world->Step(1);
      double curV = joint->GetVelocity(0);
      double accel = (curV - lastV) / dt;
      gzdbg << i << " : " << curV << " : " << (curV - lastV) / dt << "\n";
      lastV = curV;
      EXPECT_NEAR(accel, torque / link->GetInertial()->GetIXX(), TOL);
    }
  }

  {
    // get model
    physics::ModelPtr model = world->GetModel("model_2");
    ASSERT_TRUE(model != NULL);
    physics::LinkPtr link = model->GetLink("link_1");
    ASSERT_TRUE(link != NULL);
    physics::JointPtr joint = model->GetJoint("joint_01");

    double lastV = 0;
    double dt = world->GetPhysicsEngine()->GetMaxStepSize();
    for (unsigned int i = 0; i < 10; ++i)
    {
      double torque = 1.3;
      joint->SetForce(0, torque);
      world->Step(1);
      double curV = joint->GetVelocity(0);
      double accel = (curV - lastV) / dt;
      gzdbg << i << " : " << curV << " : " << (curV - lastV) / dt << "\n";
      lastV = curV;
      EXPECT_NEAR(accel, torque / link->GetInertial()->GetIZZ(), TOL);
    }
  }
}

TEST_P(JointForceTorqueTest, ForceTorque1)
{
  ForceTorque1(GetParam());
}

TEST_P(JointForceTorqueTest, ForceTorque2)
{
  ForceTorque2(GetParam());
}

TEST_P(JointForceTorqueTest, GetForceTorqueWithAppliedForce)
{
  GetForceTorqueWithAppliedForce(GetParam());
}

TEST_P(JointForceTorqueTest, JointTorqueTest)
{
  JointTorqueTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointForceTorqueTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
