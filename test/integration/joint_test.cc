/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/physics/ScrewJoint.hh"
#include "test/integration/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

#define TOL 1e-6
#define TOL_CONT 2.0

using namespace gazebo;

void Joint_TEST::ScrewJoint1(const std::string &_physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Screw Joint is not yet working.  See issue #992.\n";
    return;
  }
  // Load our screw joint test world
  Load("worlds/screw_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(math::Vector3(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();


  /// \TODO: FIXME: simbody loses joint limits after resetting simulation
  /// Add a test to check joint limits after calling Joint::Reset()


  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get model, joints and get links
  physics::ModelPtr model_1 = world->GetModel("model_1");
  physics::LinkPtr link_00 = model_1->GetLink("link_00");
  physics::LinkPtr link_01 = model_1->GetLink("link_01");
  physics::JointPtr joint_00 = model_1->GetJoint("joint_00");
  physics::JointPtr joint_01 = model_1->GetJoint("joint_01");

  // both initial angles should be zero
  EXPECT_EQ(joint_00->GetAngle(0), 0);
  EXPECT_EQ(joint_00->GetAngle(1), 0);

  // move child link to it's initial location
  link_00->SetWorldPose(math::Pose(0, 0, 2, 0, 0, 0));
  EXPECT_EQ(joint_00->GetAngle(0), 0);
  EXPECT_EQ(joint_00->GetAngle(1), 0);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(1, 0, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "]\n";

  // move child link 45deg about x
  double pitch_00 = joint_00->GetAttribute("thread_pitch", 0);
  math::Pose pose_00 = math::Pose(0.25*M_PI*pitch_00, 0, 2, 0.25*M_PI, 0, 0);
  math::Pose pose_01 = math::Pose(0, 0, -1, 0, 0, 0) + pose_00;
  link_00->SetWorldPose(pose_00);
  link_01->SetWorldPose(pose_01);
  EXPECT_EQ(joint_00->GetAngle(0), 0.25*M_PI);
  EXPECT_EQ(joint_00->GetAngle(1), 0.25*M_PI*pitch_00);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(1, 0, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "] pitch_00 [" << pitch_00
        << "]\n";

  // move child link 45deg about y
  double pitch_01 = joint_01->GetAttribute("thread_pitch", 0);
  link_00->SetWorldPose(math::Pose(0, 0, 2, 0, 0.25*M_PI, 0));
  pose_00 = math::Pose(0.25*M_PI*pitch_00, 0, 2, 0.25*M_PI, 0, 0);
  pose_01 = math::Pose(0.3*M_PI*pitch_01, 0, -1, 0.3*M_PI, 0, 0) + pose_00;
  link_00->SetWorldPose(pose_00);
  link_01->SetWorldPose(pose_01);
  EXPECT_EQ(joint_00->GetAngle(0), 0.25*M_PI);
  EXPECT_EQ(joint_00->GetAngle(1), 0.25*M_PI*pitch_00);
  EXPECT_EQ(joint_01->GetAngle(0), 0.3*M_PI);
  EXPECT_EQ(joint_01->GetAngle(1), 0.3*M_PI*pitch_01);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(1, 0, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "] pitch_00 [" << pitch_00
        << "] pitch_01 [" << pitch_01
        << "]\n";

  // new poses should not violate the constraint.  take a few steps
  // and make sure nothing moves.
  world->Step(10);

  // move child link 90deg about both x and "rotated y axis" (z)
  EXPECT_EQ(joint_00->GetAngle(0), 0.25*M_PI);
  EXPECT_EQ(joint_00->GetAngle(1), 0.25*M_PI*pitch_00);
  EXPECT_EQ(joint_01->GetAngle(0), 0.3*M_PI);
  EXPECT_EQ(joint_01->GetAngle(1), 0.3*M_PI*pitch_01);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(1, 0, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "] pitch_00 [" << pitch_00
        << "] pitch_01 [" << pitch_01
        << "]\n";
}

TEST_P(Joint_TEST, ScrewJoint1)
{
  ScrewJoint1(this->physicsEngine);
}

//////////////////////////////////////////////////
void JointTest::JointCreationDestructionTest(const std::string &_physicsEngine)
{
  /// \TODO: Disable for now until functionality is implemented
  /// bullet collision parameters needs tweaking
  if (_physicsEngine == "bullet")
  {
    gzerr << "Aborting test for bullet, see issue #590.\n";
    return;
  }
  /// \TODO: simbody not complete for this test
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #862.\n";
    return;
  }
  /// \TODO: dart not complete for this test
  if (_physicsEngine == "dart")
  {
    gzerr << "Aborting test for DART, see issue #903.\n";
    return;
  }

  // Load our inertial test world
  Load("worlds/joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

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

  // The memory footprint on osx can take around 190 cycles to stabilize.
  // So this test gives 250 cycles to stabilize and then verifies stability
  // for another 250.
  unsigned int cyclesMax = 500;
  unsigned int cyclesStabilize = cyclesMax / 2;
  for (unsigned int i = 0; i < cyclesMax; ++i)
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

    world->Step(200);

    this->GetMemInfo(residentCur, shareCur);

    // give it 2 cycles to stabilize
    if (i > cyclesStabilize)
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

//////////////////////////////////////////////////
void JointTest::SpringDamperTest(const std::string &_physicsEngine)
{
  /// SpringDamper implemented not yet released for dart
  if (_physicsEngine == "dart")
  {
    gzerr << "Aborting test for dart, see issue #975.\n";
    return;
  }
  /// SpringDamper unimplemented for simbody
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for simbody, see issue #886.\n";
    return;
  }
  /// bullet collision parameters needs tweaking
  if (_physicsEngine == "bullet")
  {
    gzerr << "Aborting test for bullet, see issue #887.\n";
    return;
  }

  // Load our inertial test world
  Load("worlds/spring_damper_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // All models should oscillate with the same frequency
  physics::ModelPtr modelPrismatic = world->GetModel("model_3_prismatic");
  physics::ModelPtr modelRevolute = world->GetModel("model_3_revolute");
  physics::ModelPtr modelPlugin = world->GetModel("model_4_prismatic_plugin");
  physics::ModelPtr modelContact = world->GetModel("model_5_soft_contact");

  ASSERT_TRUE(modelPrismatic != NULL);
  ASSERT_TRUE(modelRevolute != NULL);
  ASSERT_TRUE(modelPlugin != NULL);
  ASSERT_TRUE(modelContact != NULL);

  physics::LinkPtr linkPrismatic = modelPrismatic->GetLink("link_1");
  physics::LinkPtr linkRevolute = modelRevolute->GetLink("link_1");
  physics::LinkPtr linkPluginExplicit = modelPlugin->GetLink("link_1");
  physics::LinkPtr linkPluginImplicit = modelPlugin->GetLink("link_2");
  physics::LinkPtr linkContact = modelContact->GetLink("link_1");

  ASSERT_TRUE(linkPrismatic != NULL);
  ASSERT_TRUE(linkRevolute != NULL);
  ASSERT_TRUE(linkPluginExplicit != NULL);
  ASSERT_TRUE(linkPluginImplicit != NULL);
  ASSERT_TRUE(linkContact != NULL);

  int cyclesPrismatic = 0;
  int cyclesRevolute = 0;
  int cyclesPluginExplicit = 0;
  int cyclesPluginImplicit = 0;
  int cyclesContact = 0;

  double velPrismatic = 1.0;
  double velRevolute = 1.0;
  double velPluginExplicit = 1.0;
  double velPluginImplicit = 1.0;
  double velContact = 1.0;
  const double vT = 0.01;

  // check number of oscillations for each of the setup.  They should all
  // be the same.
  // run 5000 steps, at which point, contact is the first one to damp out
  // and lose it's oscillatory behavior due to larger dissipation in
  // contact behavior.
  for (int i = 0; i < 5000; ++i)
  {
    world->Step(1);

    // count up and down cycles
    if (linkPrismatic->GetWorldLinearVel().z > vT && velPrismatic < -vT)
    {
      cyclesPrismatic++;
      velPrismatic = 1.0;
    }
    else if (linkPrismatic->GetWorldLinearVel().z < -vT && velPrismatic > vT)
    {
      cyclesPrismatic++;
      velPrismatic = -1.0;
    }
    if (-linkRevolute->GetRelativeAngularVel().y > vT && velRevolute < -vT)
    {
      cyclesRevolute++;
      velRevolute = 1.0;
    }
    else if (-linkRevolute->GetRelativeAngularVel().y < -vT && velRevolute > vT)
    {
      cyclesRevolute++;
      velRevolute = -1.0;
    }
    if (linkPluginExplicit->GetWorldLinearVel().z > vT &&
        velPluginExplicit < -vT)
    {
      cyclesPluginExplicit++;
      velPluginExplicit = 1.0;
    }
    else if (linkPluginExplicit->GetWorldLinearVel().z < -vT &&
             velPluginExplicit > vT)
    {
      cyclesPluginExplicit++;
      velPluginExplicit = -1.0;
    }
    if (linkPluginImplicit->GetWorldLinearVel().z > vT &&
             velPluginImplicit < -vT)
    {
      cyclesPluginImplicit++;
      velPluginImplicit = 1.0;
    }
    else if (linkPluginImplicit->GetWorldLinearVel().z < -vT &&
             velPluginImplicit > vT)
    {
      cyclesPluginImplicit++;
      velPluginImplicit = -1.0;
    }
    if (linkContact->GetWorldLinearVel().z > vT && velContact < -vT)
    {
      cyclesContact++;
      velContact = 1.0;
    }
    else if (linkContact->GetWorldLinearVel().z < -vT && velContact > vT)
    {
      cyclesContact++;
      velContact = -1.0;
    }

    // gzdbg << i << "\n";
    // gzdbg << cyclesPrismatic << " : "
    //       << linkPrismatic->GetWorldLinearVel() << "\n";
    // gzdbg << cyclesRevolute << " : "
    //       << linkRevolute->GetRelativeAngularVel() << "\n";
    // gzdbg << cyclesContact << " : "
    //       << linkContact->GetWorldLinearVel() << "\n";
  }
  EXPECT_EQ(cyclesPrismatic,      17);
  EXPECT_EQ(cyclesRevolute,       17);
  EXPECT_EQ(cyclesPluginExplicit, 17);
  EXPECT_EQ(cyclesPluginImplicit, 17);
  EXPECT_EQ(cyclesContact,        17);
}

//////////////////////////////////////////////////
TEST_F(JointTest, joint_SDF14)
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

TEST_P(JointTest, JointCreationDestructionTest)
{
  JointCreationDestructionTest(this->physicsEngine);
}

TEST_P(JointTest, SpringDamperTest)
{
  SpringDamperTest(this->physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("")));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
