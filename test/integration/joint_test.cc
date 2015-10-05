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
#include "gazebo/test/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

#define TOL 1e-6
#define TOL_CONT 2.0

using namespace gazebo;

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
void JointTest::GetInertiaRatio(const std::string &_physicsEngine)
{
  // Load our inertia ratio world
  Load("worlds/inertia_ratio.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics::ModelPtr model = world->GetModel("double_pendulum");
  ASSERT_TRUE(model != NULL);

  {
    physics::JointPtr joint = model->GetJoint("lower_joint");
    ASSERT_TRUE(joint != NULL);

    EXPECT_NEAR(joint->GetInertiaRatio(0), 3125, 1e-2);
    EXPECT_NEAR(joint->GetInertiaRatio(math::Vector3::UnitX), 3125, 1e-2);
    EXPECT_NEAR(joint->GetInertiaRatio(math::Vector3::UnitY), 87.50, 1e-2);
  }
}

//////////////////////////////////////////////////
void JointTest::SpringDamperTest(const std::string &_physicsEngine)
{
  /// bullet collision parameters needs tweaking
  if (_physicsEngine == "bullet")
  {
    gzerr << "Aborting test for bullet, see issue #887.\n";
    return;
  }
  if (_physicsEngine == "dart")
  {
    gzerr << "Aborting test for " << _physicsEngine
          << ", see issue #975.\n";
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
  physics::ModelPtr modelPrismatic2 = world->GetModel("model_6_prismatic_sdf");
  physics::ModelPtr modelRevolute2 = world->GetModel("model_7_revolute_sdf");

  ASSERT_TRUE(modelPrismatic != NULL);
  ASSERT_TRUE(modelRevolute != NULL);
  ASSERT_TRUE(modelPlugin != NULL);
  ASSERT_TRUE(modelContact != NULL);
  ASSERT_TRUE(modelPrismatic2 != NULL);
  ASSERT_TRUE(modelRevolute2 != NULL);

  physics::LinkPtr linkPrismatic = modelPrismatic->GetLink("link_1");
  physics::LinkPtr linkRevolute = modelRevolute->GetLink("link_1");
  physics::LinkPtr linkPluginExplicit = modelPlugin->GetLink("link_1");
  physics::LinkPtr linkPluginImplicit = modelPlugin->GetLink("link_2");
  physics::LinkPtr linkContact = modelContact->GetLink("link_1");
  physics::LinkPtr linkPrismatic2 = modelPrismatic2->GetLink("link");
  physics::LinkPtr linkRevolute2 = modelRevolute2->GetLink("link");

  ASSERT_TRUE(linkPrismatic != NULL);
  ASSERT_TRUE(linkRevolute != NULL);
  ASSERT_TRUE(linkPluginExplicit != NULL);
  ASSERT_TRUE(linkPluginImplicit != NULL);
  ASSERT_TRUE(linkContact != NULL);
  ASSERT_TRUE(linkPrismatic2 != NULL);
  ASSERT_TRUE(linkRevolute2 != NULL);

  physics::JointPtr jointPluginImplicit = modelPlugin->GetJoint("joint_1");
  ASSERT_TRUE(jointPluginImplicit != NULL);

  int cyclesPrismatic = 0;
  int cyclesRevolute = 0;
  int cyclesPluginExplicit = 0;
  int cyclesPluginImplicit = 0;
  int cyclesContact = 0;
  int cyclesPrismatic2 = 0;
  int cyclesRevolute2 = 0;

  double velPrismatic = 1.0;
  double velRevolute = 1.0;
  double velPluginExplicit = 1.0;
  double velPluginImplicit = 1.0;
  double velContact = 1.0;
  double velPrismatic2 = 1.0;
  double velRevolute2 = 1.0;
  const double vT = 0.01;

  double energyPluginImplicit0 = linkPluginImplicit->GetWorldEnergy()
        + jointPluginImplicit->GetWorldEnergyPotentialSpring(0);

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
    if (linkPrismatic2->GetWorldLinearVel().z > vT && velPrismatic2 < -vT)
    {
      cyclesPrismatic2++;
      velPrismatic2 = 1.0;
    }
    else if (linkPrismatic2->GetWorldLinearVel().z < -vT && velPrismatic2 > vT)
    {
      cyclesPrismatic2++;
      velPrismatic2 = -1.0;
    }
    if (-linkRevolute2->GetRelativeAngularVel().y > vT && velRevolute2 < -vT)
    {
      cyclesRevolute2++;
      velRevolute2 = 1.0;
    }
    else if (-linkRevolute2->GetRelativeAngularVel().y < -vT &&
             velRevolute2 > vT)
    {
      cyclesRevolute2++;
      velRevolute2 = -1.0;
    }

    double energy = linkPluginImplicit->GetWorldEnergy() +
                   jointPluginImplicit->GetWorldEnergyPotentialSpring(0);
    if (_physicsEngine.compare("dart") == 0)
    {
      if (i == 0)
      {
        gzerr << _physicsEngine
              << " has reduced accuracy for spring energy conservation"
              << ", see #975"
              << std::endl;
      }
      EXPECT_NEAR(energy / energyPluginImplicit0, 1.0, 2e-2);
    }
    else
    {
      EXPECT_NEAR(energy / energyPluginImplicit0, 1.0, 1e-3);
    }
    // gzdbg << i << "\n";
    // gzdbg << cyclesPrismatic << " : "
    //       << linkPrismatic->GetWorldLinearVel() << "\n";
    // gzdbg << cyclesRevolute << " : "
    //       << linkRevolute->GetRelativeAngularVel() << "\n";
    // gzdbg << cyclesContact << " : "
    //       << linkContact->GetWorldLinearVel() << "\n";
  }
  if (_physicsEngine.compare("ode") == 0)
  {
    gzdbg << "Extra tests for ode" << std::endl;
    EXPECT_EQ(cyclesContact,        17);
  }
  if (_physicsEngine.compare("dart") == 0)
  {
    gzerr << _physicsEngine
          << " doesn't support joint limit stiffness"
          << ", see #975"
          << std::endl;
  }
  else
  {
    EXPECT_EQ(cyclesPrismatic,      17);
    EXPECT_EQ(cyclesRevolute,       17);
  }
  EXPECT_EQ(cyclesPluginExplicit, 17);
  EXPECT_EQ(cyclesPluginImplicit, 17);
  EXPECT_EQ(cyclesPrismatic2,     17);
  EXPECT_EQ(cyclesRevolute2,      17);
}

//////////////////////////////////////////////////
void JointTest::DynamicJointVisualization(const std::string &_physicsEngine)
{
  // Load empty world
  Load("worlds/empty.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Spawn two boxes
  SpawnBox("box1", math::Vector3(1, 1, 1), math::Vector3(1, 0, 0.5),
      math::Vector3::Zero, false);
  SpawnBox("box2", math::Vector3(1, 1, 1), math::Vector3(-1, 0, 0.5),
      math::Vector3::Zero, false);
  physics::ModelPtr model  = world->GetModel("box1");
  physics::ModelPtr model2 = world->GetModel("box2");
  ASSERT_TRUE(model  != NULL);
  ASSERT_TRUE(model2 != NULL);

  // Get link pointers
  physics::LinkPtr parent = model->GetLink();
  physics::LinkPtr child  = model2->GetLink();

  // Create dynamic joint
  std::string name = "dynamic_joint_unique";
  physics::JointPtr joint;
  joint = model->CreateJoint(name, "revolute", parent, child);
  joint->Init();

  // Get model joints (used for visualization)
  physics::Joint_V joints = model->GetJoints();
  physics::Joint_V::iterator iter;
  bool jointFound = false;
  for (iter = joints.begin(); iter != joints.end(); ++iter)
  {
    if ((*iter)->GetName() == name)
    {
      jointFound = true;
      break;
    }
  }
  EXPECT_TRUE(jointFound);
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
  EXPECT_TRUE(physicsEngine != NULL);
  physics::ModelPtr model = world->GetModel("joint14_model");
  EXPECT_TRUE(model != NULL);
  physics::LinkPtr link1 = model->GetLink("body1");
  EXPECT_TRUE(link1 != NULL);
  physics::LinkPtr link2 = model->GetLink("body2");
  EXPECT_TRUE(link2 != NULL);

  EXPECT_EQ(model->GetJointCount(), 1u);
  physics::JointPtr joint = model->GetJoint("joint14_revolute_joint");
  EXPECT_TRUE(joint != NULL);

  physics::LinkPtr parent = joint->GetParent();
  EXPECT_TRUE(parent != NULL);
  physics::LinkPtr child = joint->GetChild();
  EXPECT_TRUE(child != NULL);
  EXPECT_EQ(parent->GetName(), "body2");
  EXPECT_EQ(child->GetName(), "body1");
}

TEST_P(JointTest, JointCreationDestructionTest)
{
  JointCreationDestructionTest(this->physicsEngine);
}

TEST_P(JointTest, GetInertiaRatio)
{
  GetInertiaRatio(this->physicsEngine);
}

TEST_P(JointTest, SpringDamperTest)
{
  SpringDamperTest(this->physicsEngine);
}

TEST_P(JointTest, DynamicJointVisualization)
{
  DynamicJointVisualization(this->physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("")));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
