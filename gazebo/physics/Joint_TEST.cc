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
#include "gazebo/physics/Joint.hh"

#define TOL 1e-6
#define TOL_CONT 2.0

using namespace gazebo;

class Joint_TEST : public ServerFixture
{
};

////////////////////////////////////////////////////////////////////////
// Load example world with a few joints
// Measure force torques
// with active torque control at joints
////////////////////////////////////////////////////////////////////////
TEST_F(Joint_TEST, GetForceTorqueWithAppliedForce)
{
  // Load our force torque test world
  Load("worlds/force_torque_demo2.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), "simbody");

  physics->SetGravity(math::Vector3(0, 0, -50));

  // simulate 1 step
  world->StepWorld(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzdbg << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_GT(t, 0);
  gzdbg << "t after one step : " << t << "\n";

  // get joint and get force torque
  physics::ModelPtr model_1 = world->GetModel("boxes");
  physics::JointPtr joint_01 = model_1->GetJoint("joint1");
  physics::JointPtr joint_12 = model_1->GetJoint("joint2");

  gzdbg << "------------------- PD CONTROL -------------------\n";
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

    world->StepWorld(1);
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

      gzdbg << "joint_01 force torque : "
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

      gzdbg << "joint_12 force torque : "
            << "step [" << i
            << "] GetForce [" << joint_12->GetForce(0u)
            << "] command [" << effort2
            << "] force1 [" << wrench_12.body1Force
            << "] torque1 [" << wrench_12.body1Torque
            << "] force2 [" << wrench_12.body2Force
            << "] torque2 [" << wrench_12.body2Torque
            << "]\n";
    }
  }
}

////////////////////////////////////////////////////////////////////////
// Create a joint between link and world
// Apply force and check acceleration for correctness
////////////////////////////////////////////////////////////////////////
TEST_F(Joint_TEST, JointTorqueTest)
{
  // Load our inertial test world
  Load("worlds/joint_test.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), "ode");

  // create some fake links
  physics::ModelPtr model = world->GetModel("model_1");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink("link_1");
  ASSERT_TRUE(link != NULL);

  physics::LinkPtr parentLink;
  physics::LinkPtr childLink(link);
  physics::JointPtr joint;
  math::Pose anchor;
  double upper = M_PI;
  double lower = -M_PI;

  {
    // create a joint
    {
      joint = world->GetPhysicsEngine()->CreateJoint(
        "revolute", model);
      joint->Attach(parentLink, childLink);
      // load adds the joint to a vector of shared pointers kept
      // in parent and child links, preventing joint from being destroyed.
      joint->Load(parentLink, childLink, anchor);
      // joint->SetAnchor(0, anchor);
      joint->SetAxis(0, math::Vector3(1, 0, 0));
      joint->SetHighStop(0, upper);
      joint->SetLowStop(0, lower);

      if (parentLink)
        joint->SetName(parentLink->GetName() + std::string("_") +
                       childLink->GetName() + std::string("_joint"));
      else
        joint->SetName(std::string("world_") +
                       childLink->GetName() + std::string("_joint"));
      joint->Init();
    }

    double lastV = 0;
    double dt = world->GetPhysicsEngine()->GetMaxStepSize();
    for (unsigned int i = 0; i < 10; ++i)
    {
      double torque = 1.3;
      joint->SetForce(0, torque);
      world->StepWorld(1);
      double curV = joint->GetVelocity(0);
      double accel = (curV - lastV) / dt;
      gzdbg << i << " : " << curV << " : " << (curV - lastV) / dt << "\n";
      lastV = curV;
      EXPECT_NEAR(accel, torque / link->GetInertial()->GetIXX(), TOL);
    }

    // remove the joint
    {
      bool paused = world->IsPaused();
      world->SetPaused(true);
      if (joint)
      {
        // reenable collision between the link pair
        physics::LinkPtr parent = joint->GetParent();
        physics::LinkPtr child = joint->GetChild();
        if (parent)
          parent->SetCollideMode("all");
        if (child)
          child->SetCollideMode("all");

        joint->Detach();
        joint.reset();
      }
      world->SetPaused(paused);
    }
  }

  {
    // create a joint
    {
      joint = world->GetPhysicsEngine()->CreateJoint(
        "revolute", model);
      joint->Attach(parentLink, childLink);
      // load adds the joint to a vector of shared pointers kept
      // in parent and child links, preventing joint from being destroyed.
      joint->Load(parentLink, childLink, anchor);
      // joint->SetAnchor(0, anchor);
      joint->SetAxis(0, math::Vector3(0, 0, 1));
      joint->SetHighStop(0, upper);
      joint->SetLowStop(0, lower);

      if (parentLink)
        joint->SetName(parentLink->GetName() + std::string("_") +
                       childLink->GetName() + std::string("_joint"));
      else
        joint->SetName(std::string("world_") +
                       childLink->GetName() + std::string("_joint"));
      joint->Init();
    }

    double lastV = 0;
    double dt = world->GetPhysicsEngine()->GetMaxStepSize();
    for (unsigned int i = 0; i < 10; ++i)
    {
      double torque = 1.3;
      joint->SetForce(0, torque);
      world->StepWorld(1);
      double curV = joint->GetVelocity(0);
      double accel = (curV - lastV) / dt;
      gzdbg << i << " : " << curV << " : " << (curV - lastV) / dt << "\n";
      lastV = curV;
      EXPECT_NEAR(accel, torque / link->GetInertial()->GetIZZ(), TOL);
    }

    // remove the joint
    {
      bool paused = world->IsPaused();
      world->SetPaused(true);
      if (joint)
      {
        // reenable collision between the link pair
        physics::LinkPtr parent = joint->GetParent();
        physics::LinkPtr child = joint->GetChild();
        if (parent)
          parent->SetCollideMode("all");
        if (child)
          child->SetCollideMode("all");

        joint->Detach();
        joint.reset();
      }
      world->SetPaused(paused);
    }
  }
}

TEST_F(Joint_TEST, JointCreationDestructionTest)
{
  // Load our inertial test world
  Load("worlds/joint_test.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), "ode");

  // create some fake links
  physics::ModelPtr model = world->GetModel("model_1");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink("link_1");
  ASSERT_TRUE(link != NULL);

  physics::LinkPtr parentLink;
  physics::LinkPtr childLink(link);
  physics::JointPtr joint;
  math::Pose anchor;
  math::Vector3 axis(1, 0, 0);
  double upper = M_PI;
  double lower = -M_PI;

  double residentLast = 0, shareLast = 0;
  double residentCur = 0, shareCur = 0;

  for (unsigned int i = 0; i < 100; ++i)
  {
    // try creating a joint
    {
      joint = world->GetPhysicsEngine()->CreateJoint(
        "revolute", model);
      joint->Attach(parentLink, childLink);
      // load adds the joint to a vector of shared pointers kept
      // in parent and child links, preventing joint from being destroyed.
      joint->Load(parentLink, childLink, anchor);
      // joint->SetAnchor(0, anchor);
      joint->SetAxis(0, axis);
      joint->SetHighStop(0, upper);
      joint->SetLowStop(0, lower);

      if (parentLink)
        joint->SetName(parentLink->GetName() + std::string("_") +
                       childLink->GetName() + std::string("_joint"));
      else
        joint->SetName(std::string("world_") +
                       childLink->GetName() + std::string("_joint"));
      joint->Init();
      joint->SetAxis(0, axis);
    }
    // remove the joint
    {
      bool paused = world->IsPaused();
      world->SetPaused(true);
      if (joint)
      {
        // reenable collision between the link pair
        physics::LinkPtr parent = joint->GetParent();
        physics::LinkPtr child = joint->GetChild();
        if (parent)
          parent->SetCollideMode("all");
        if (child)
          child->SetCollideMode("all");

        joint->Detach();
        joint.reset();
      }
      world->SetPaused(paused);
    }

    gazebo::common::Time::MSleep(10);

    this->GetMemInfo(residentCur, shareCur);
    if (i > 1)  // give it 2 cycles to stabilize
    {
      EXPECT_LE(residentCur, residentLast);
      EXPECT_LE(shareCur, shareLast);
    }
    // gzdbg << "memory res[" << residentCur
    //       << "] shr[" << shareCur
    //       << "] res[" << residentLast
    //       << "] shr[" << shareLast
    //       << "]\n";
    residentLast = residentCur;
    shareLast = shareCur;
  }
}

TEST_F(Joint_TEST, joint_SDF14)
{
  Load("worlds/SDF_1_4.world");

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);

  int i = 0;
  while (!this->HasEntity("joint14_model") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get joint14_model");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();
  EXPECT_TRUE(physicsEngine);
  physics::ModelPtr model = world->GetModel("joint14_model");
  EXPECT_TRUE(model);
  physics::LinkPtr link1 = model->GetLink("body1");
  EXPECT_TRUE(link1);
  physics::LinkPtr link2 = model->GetLink("body2");
  EXPECT_TRUE(link2);

  EXPECT_EQ(model->GetJointCount(), 1u);
  physics::JointPtr joint = model->GetJoint("joint14_revolute_joint");
  EXPECT_TRUE(joint);

  physics::LinkPtr parent = joint->GetParent();
  EXPECT_TRUE(parent);
  physics::LinkPtr child = joint->GetChild();
  EXPECT_TRUE(child);
  EXPECT_EQ(parent->GetName(), "body2");
  EXPECT_EQ(child->GetName(), "body1");
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
