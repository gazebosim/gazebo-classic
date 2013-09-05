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
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Joint_TEST.hh"
#include "test/integration/helper_physics_generator.hh"

#define TOL 1e-6
using namespace gazebo;

// Fixture for testing all joint types.
class Joint_TEST_All : public Joint_TEST {};

// Fixture for testing rotational joints.
class Joint_TEST_Rotational : public Joint_TEST {};

////////////////////////////////////////////////////////////////////////
// Test for spawning each joint type
void Joint_TEST::SpawnJointTypes(const std::string &_physicsEngine,
                                 const std::string &_jointType)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics::JointPtr joint;
  gzdbg << "SpawnJoint " << _jointType << " child parent" << std::endl;
  joint = SpawnJoint(_jointType, false, false);
  EXPECT_TRUE(joint != NULL);

  gzdbg << "SpawnJoint " << _jointType << " child world" << std::endl;
  joint = SpawnJoint(_jointType, false, true);
  EXPECT_TRUE(joint != NULL);

  gzdbg << "SpawnJoint " << _jointType << " world parent" << std::endl;
  joint = SpawnJoint(_jointType, true, false);
  EXPECT_TRUE(joint != NULL);
}

TEST_P(Joint_TEST_All, SpawnJointTypes)
{
  SpawnJointTypes(this->physicsEngine, this->jointType);
}

INSTANTIATE_TEST_CASE_P(TestRuns, Joint_TEST_All, ::testing::Combine(
  PHYSICS_ENGINE_VALUES,
  ::testing::Values("revolute"
                  , "prismatic"
                  , "screw"
                  , "universal"
                  , "ball"
                  , "revolute2"
                    )));

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
