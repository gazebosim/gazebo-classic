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
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

#define TOL 1e-6
#define TOL_CONT 2.0

using namespace gazebo;

const double g_tolerance = 1e-4;

class JointSpawningTest : public JointTest
{
  /// \brief Spawn model with each type of joint.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _jointType Type of joint to spawn and test.
  public: void SpawnJointTypes(const std::string &_physicsEngine,
                               const std::string &_jointType);

  /// \brief Spawn model with rotational joints. Set velocity on parent
  ///        and make sure child follows.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _jointType Type of joint to spawn and test.
  public: void SpawnJointRotational(const std::string &_physicsEngine,
                                    const std::string &_jointType);

  /// \brief Spawn model with rotational joints. Attach to world and make
  ///        sure it doesn't fall.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _jointType Type of joint to spawn and test.
  public: void SpawnJointRotationalWorld(const std::string &_physicsEngine,
                                         const std::string &_jointType);

  /// \brief Check joint properties.
  /// \param[in] _index Joint angle index.
  /// \param[in] _joint Joint to check.
  public: void CheckJointProperties(unsigned int _index,
                                    physics::JointPtr _joint);
};

// Fixture for testing all joint types.
class JointSpawningTest_All : public JointSpawningTest {};

// Fixture for testing rotational joints.
class JointSpawningTest_Rotational : public JointSpawningTest {};

// Fixture for testing rotational joints that can be attached to world.
class JointSpawningTest_RotationalWorld : public JointSpawningTest {};

////////////////////////////////////////////////////////////////////////
// Test for spawning each joint type
void JointSpawningTest::SpawnJointTypes(const std::string &_physicsEngine,
                                        const std::string &_jointType)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // disable gravity
  physics->SetGravity(ignition::math::Vector3d::Zero);

  {
    gzdbg << "SpawnJoint " << _jointType << " child parent" << std::endl;
    physics::JointPtr joint = SpawnJoint(_jointType, false, false);
    ASSERT_TRUE(joint != nullptr);
    // Check child and parent links
    physics::LinkPtr child = joint->GetChild();
    physics::LinkPtr parent = joint->GetParent();
    ASSERT_TRUE(child != nullptr);
    EXPECT_EQ(child->GetParentJoints().size(), 1u);
    EXPECT_EQ(child->GetChildJoints().size(), 0u);
    ASSERT_TRUE(parent != nullptr);
    EXPECT_EQ(parent->GetChildJoints().size(), 1u);
    EXPECT_EQ(parent->GetParentJoints().size(), 0u);
    EXPECT_EQ(_jointType, msgs::ConvertJointType(joint->GetMsgType()));
    for (unsigned int i = 0; i < joint->DOF(); ++i)
    {
      CheckJointProperties(i, joint);
    }
  }

  // child link with off-diagonal inertias
  // skip universal joints because the off-diagonal inertias
  // add coupling between the axes, which messes up CheckJointProperties
  if (_jointType != "universal")
  {
    gzdbg << "SpawnJoint " << _jointType
          << " child with off-diagonal inertia, parent"
          << std::endl;
    SpawnJointOptions opt;
    opt.type = _jointType;
    opt.useChildLinkInertia = true;
    EXPECT_TRUE(opt.childLinkInertia.SetMassMatrix(
      ignition::math::MassMatrix3d(12.0,
        ignition::math::Vector3d(2.0, 3.0, 4.0),
        ignition::math::Vector3d(0, 0, 0.4))));
    physics::JointPtr joint = SpawnJoint(opt);
    ASSERT_TRUE(joint != nullptr);
    // Check child and parent links
    physics::LinkPtr child = joint->GetChild();
    physics::LinkPtr parent = joint->GetParent();
    ASSERT_TRUE(child != nullptr);
    EXPECT_EQ(child->GetParentJoints().size(), 1u);
    EXPECT_EQ(child->GetChildJoints().size(), 0u);
    ASSERT_TRUE(parent != nullptr);
    EXPECT_EQ(parent->GetChildJoints().size(), 1u);
    EXPECT_EQ(parent->GetParentJoints().size(), 0u);
    EXPECT_EQ(_jointType, msgs::ConvertJointType(joint->GetMsgType()));
    for (unsigned int i = 0; i < joint->DOF(); ++i)
    {
      CheckJointProperties(i, joint);
    }
  }

  if (_jointType == "gearbox")
  {
    gzerr << "Skip connect to world tests, since we aren't specifying "
          << "the reference body."
          << std::endl;
    return;
  }

  {
    gzdbg << "SpawnJoint " << _jointType << " child world" << std::endl;
    physics::JointPtr joint = SpawnJoint(_jointType, false, true);
    ASSERT_TRUE(joint != nullptr);
    // Check child link
    physics::LinkPtr child = joint->GetChild();
    physics::LinkPtr parent = joint->GetParent();
    ASSERT_TRUE(child != nullptr);
    EXPECT_EQ(child->GetParentJoints().size(), 1u);
    EXPECT_EQ(child->GetChildJoints().size(), 0u);
    EXPECT_TRUE(parent == nullptr);
    for (unsigned int i = 0; i < joint->DOF(); ++i)
    {
      CheckJointProperties(i, joint);
    }
  }

  if (_physicsEngine == "dart")
  {
    // DART assumes that: (i) every link has its parent joint (ii) root link
    // is the only link that doesn't have parent link.
    // Child world link breaks dart for now. Do we need to support it?
    gzerr << "Skip tests for child world link cases "
          << "since DART does not allow joint with world as child. "
          << "Please see issue #914. "
          << "(https://bitbucket.org/osrf/gazebo/issue/914)"
          << std::endl;
  }
  else
  {
    gzdbg << "SpawnJoint " << _jointType << " world parent" << std::endl;
    physics::JointPtr joint = SpawnJoint(_jointType, true, false);
    ASSERT_TRUE(joint != nullptr);
    // Check parent link
    physics::LinkPtr child = joint->GetChild();
    physics::LinkPtr parent = joint->GetParent();
    EXPECT_TRUE(child == nullptr);
    ASSERT_TRUE(parent != nullptr);
    EXPECT_EQ(parent->GetChildJoints().size(), 1u);
    EXPECT_EQ(parent->GetParentJoints().size(), 0u);
    for (unsigned int i = 0; i < joint->DOF(); ++i)
    {
      CheckJointProperties(i, joint);
    }
  }
}

////////////////////////////////////////////////////////////////////////
// Test for non-translational joints.
// Set velocity to parent and make sure child follows.
void JointSpawningTest::SpawnJointRotational(const std::string &_physicsEngine,
                                             const std::string &_jointType)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  gzdbg << "SpawnJoint " << _jointType << std::endl;
  physics::JointPtr joint = SpawnJoint(_jointType);
  ASSERT_TRUE(joint != nullptr);

  physics::LinkPtr parent, child;
  child = joint->GetChild();
  parent = joint->GetParent();
  ASSERT_TRUE(child != nullptr);
  ASSERT_TRUE(parent != nullptr);

  ignition::math::Vector3d pos(10, 10, 10);
  ignition::math::Vector3d vel(10, 10, 10);
  parent->SetWorldPose(ignition::math::Pose3d(pos,
        ignition::math::Quaterniond::Identity));
  for (unsigned int i = 0; i < 10; ++i)
  {
    parent->SetLinearVel(vel);
    world->Step(10);
  }
  world->Step(50);
  ignition::math::Pose3d childPose = child->WorldPose();
  ignition::math::Pose3d parentPose = parent->WorldPose();
  EXPECT_TRUE(parentPose.Pos() != pos);
  EXPECT_TRUE(parentPose.Pos() != ignition::math::Vector3d::Zero);
  EXPECT_TRUE(childPose.Pos() != ignition::math::Vector3d::Zero);
  EXPECT_TRUE(childPose.Pos() == parentPose.Pos());
  EXPECT_EQ(joint->WorldPose().Pos(), joint->ParentWorldPose().Pos());
  EXPECT_EQ(joint->AnchorErrorPose().Pos(), ignition::math::Vector3d::Zero);
}

////////////////////////////////////////////////////////////////////////
// Test for non-translational joints that can attach to world.
// Attach to world and see if it doesn't fall.
void JointSpawningTest::SpawnJointRotationalWorld(
  const std::string &_physicsEngine,
  const std::string &_jointType)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics::JointPtr joint;
  for (unsigned int i = 0; i < 2; ++i)
  {
    bool worldChild = (i == 0);
    bool worldParent = (i == 1);
    std::string child = worldChild ? "world" : "child";
    std::string parent = worldParent ? "world" : "parent";
    gzdbg << "SpawnJoint " << _jointType << " "
          << child << " "
          << parent << std::endl;

    if ((_physicsEngine == "dart" || _physicsEngine == "simbody")
        && worldChild)
    {
      // These physics engines don't support world as a child link.
      // simbody https://bitbucket.org/osrf/gazebo/issue/861
      // dart https://bitbucket.org/osrf/gazebo/issue/914
      gzerr << "Skip tests for child world link cases since "
            << _physicsEngine
            << " does not allow joint with world as child. "
            << "Please see bitbucket issues #861, #914."
            << std::endl;
      continue;
    }

    joint = SpawnJoint(_jointType, worldChild, worldParent);
    ASSERT_TRUE(joint != nullptr);

    physics::LinkPtr link;
    if (!worldChild)
      link = joint->GetChild();
    else if (!worldParent)
      link = joint->GetParent();
    ASSERT_TRUE(link != nullptr);

    auto initialPose = link->WorldPose();
    world->Step(100);
    auto afterPose = link->WorldPose();
    EXPECT_TRUE(initialPose.Pos() == afterPose.Pos());
    EXPECT_EQ(joint->WorldPose().Pos(), joint->ParentWorldPose().Pos());
    EXPECT_EQ(joint->AnchorErrorPose().Pos(), ignition::math::Vector3d::Zero);
  }
}

/////////////////////////////////////////////////
void JointSpawningTest::CheckJointProperties(unsigned int _index,
                                        physics::JointPtr _joint)
{
  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != nullptr);
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  bool isBullet = physics->GetType().compare("bullet") == 0;
  bool isDart = physics->GetType().compare("dart") == 0;
  bool isSimbody = physics->GetType().compare("simbody") == 0;
  double dt = physics->GetMaxStepSize();

  if (_joint->HasType(physics::Base::GEARBOX_JOINT))
  {
    // Test setting gear ratio and reference angles
    EXPECT_TRUE(_joint->SetParam("reference_angle1", 0, 1.0));
    EXPECT_TRUE(_joint->SetParam("reference_angle2", 0, 2.0));
    EXPECT_TRUE(_joint->SetParam("ratio", 0, 3.0));

    EXPECT_DOUBLE_EQ(1.0, _joint->GetParam("reference_angle1", 0));
    EXPECT_DOUBLE_EQ(2.0, _joint->GetParam("reference_angle2", 0));
    EXPECT_DOUBLE_EQ(3.0, _joint->GetParam("ratio", 0));

    EXPECT_TRUE(_joint->SetParam("reference_angle1", 0, 4.0));
    EXPECT_TRUE(_joint->SetParam("reference_angle2", 0, 5.0));
    EXPECT_TRUE(_joint->SetParam("ratio", 0, 6.0));

    EXPECT_DOUBLE_EQ(4.0, _joint->GetParam("reference_angle1", 0));
    EXPECT_DOUBLE_EQ(5.0, _joint->GetParam("reference_angle2", 0));
    EXPECT_DOUBLE_EQ(6.0, _joint->GetParam("ratio", 0));

    gzerr << "This portion of the test fails for this joint type" << std::endl;
    return;
  }

  if (_joint->HasType(physics::Base::HINGE2_JOINT) ||
      _joint->HasType(physics::Base::SCREW_JOINT))
  {
    gzerr << "This portion of the test fails for this joint type" << std::endl;
    return;
  }
  if (!_joint->GetChild())
  {
    gzerr << "The rest of this test fails without a child link" << std::endl;
    return;
  }

  // Reset world prior to testing SetVelocity
  // This is needed for SimbodyUniversalJoint
  world->Reset();
  double velocityMagnitude = 1.0;
  std::vector<double> velocities;
  velocities.push_back(velocityMagnitude);
  velocities.push_back(0.0);
  velocities.push_back(-velocityMagnitude);
  for (std::vector<double>::iterator iter = velocities.begin();
       iter != velocities.end(); ++iter)
  {
    // Use Joint::SetVelocity with different values
    double vel = *iter;
    _joint->SetVelocity(_index, vel);

    // Verify that Joint::GetVelocity returns the same value
    EXPECT_NEAR(_joint->GetVelocity(_index), vel, g_tolerance);

    // Take some steps and verify that it keeps spinning at same speed
    world->Step(50);
    EXPECT_NEAR(_joint->GetVelocity(_index), vel, g_tolerance);
  }

  // Test SetForce with positive value
  {
    // reset world and expect joint to be stopped at home position
    world->Reset();
    EXPECT_NEAR(_joint->Position(_index), 0.0, g_tolerance);
    EXPECT_NEAR(_joint->GetVelocity(_index), 0.0, g_tolerance);

    // set positive force
    double angleStart = _joint->Position(_index);
    _joint->SetForce(_index, 5);
    world->Step(1);
    EXPECT_GT(_joint->GetVelocity(_index), 0.0);
    world->Step(1);
    EXPECT_GT(_joint->Position(_index), angleStart);
  }

  // Test SetForce with negative value
  {
    // reset world and expect joint to be stopped at home position
    world->Reset();
    EXPECT_NEAR(_joint->Position(_index), 0.0, g_tolerance);
    EXPECT_NEAR(_joint->GetVelocity(_index), 0.0, g_tolerance);

    // set negative force
    double angleStart = _joint->Position(_index);
    _joint->SetForce(_index, -5);
    world->Step(1);
    EXPECT_LT(_joint->GetVelocity(_index), 0.0);
    world->Step(1);
    EXPECT_LT(_joint->Position(_index), angleStart);
  }

  // Test Coloumb friction
  if (isBullet && _joint->HasType(physics::Base::UNIVERSAL_JOINT))
  {
    gzerr << "Skipping friction test for "
          << physics->GetType()
          << " "
          << msgs::ConvertJointType(_joint->GetMsgType())
          << " joint"
          << std::endl;
  }
  else if (isSimbody)
  {
    gzerr << "Skipping friction test for "
          << physics->GetType()
          << std::endl;
  }
  else
  {
    // reset world and expect joint to be stopped at home position
    world->Reset();
    EXPECT_NEAR(_joint->Position(_index), 0.0, g_tolerance);
    EXPECT_NEAR(_joint->GetVelocity(_index), 0.0, g_tolerance);

    // set friction to 1.0
    const double friction = 1.0;
    EXPECT_TRUE(_joint->SetParam("friction", _index, friction));
    EXPECT_NEAR(_joint->GetParam("friction", _index), friction, g_tolerance);

    for (unsigned int i = 0; i < 500; ++i)
    {
      // Apply force with smaller magnitude than friction
      _joint->SetForce(_index, friction * 0.5);
      world->Step(1);
    }

    // Expect no motion
    EXPECT_NEAR(_joint->GetVelocity(_index), 0.0, g_tolerance);
    EXPECT_NEAR(_joint->Position(_index), 0.0, g_tolerance);

    for (unsigned int i = 0; i < 500; ++i)
    {
      // Apply force with larger magnitude than friction
      _joint->SetForce(_index, friction * 1.5);
      world->Step(1);
    }

    // Expect motion
    EXPECT_GT(_joint->GetVelocity(_index), 0.2 * friction);
    EXPECT_GT(_joint->Position(_index), 0.05 * friction);

    // DART has problem with joint friction and joint limits
    // https://github.com/dartsim/dart/issues/317
    // Set friction back to zero to not interfere with other tests
    // until this issue is resolved.
    if (isDart)
    {
      EXPECT_TRUE(_joint->SetParam("friction", _index, 0.0));
      EXPECT_NEAR(_joint->GetParam("friction", _index), 0.0, g_tolerance);
    }
  }

  // SetUpperLimit
  {
    // reset world and expect joint to be stopped at home position
    world->Reset();
    EXPECT_NEAR(_joint->Position(_index), 0.0, g_tolerance);
    EXPECT_NEAR(_joint->GetVelocity(_index), 0.0, g_tolerance);

    unsigned int steps = 100;
    double vel = 1.0;
    double limit = steps * dt * vel * 0.5;
    _joint->SetUpperLimit(_index, limit);
    _joint->SetVelocity(_index, vel);
    world->Step(steps);
    EXPECT_LT(_joint->Position(_index), limit + g_tolerance);
    EXPECT_NEAR(_joint->UpperLimit(_index), limit, g_tolerance);
    {
      boost::any value = _joint->GetParam("hi_stop", _index);
      EXPECT_NEAR(boost::any_cast<double>(value), limit, g_tolerance);
    }
  }

  // SetLowerLimit
  {
    // reset world and expect joint to be stopped at home position
    world->Reset();
    EXPECT_NEAR(_joint->Position(_index), 0.0, g_tolerance);
    EXPECT_NEAR(_joint->GetVelocity(_index), 0.0, g_tolerance);

    unsigned int steps = 100;
    double vel = -1.0;
    double limit = steps * dt * vel * 0.5;
    _joint->SetLowerLimit(_index, limit);
    _joint->SetVelocity(_index, vel);
    world->Step(steps);
    EXPECT_GT(_joint->Position(_index), limit - g_tolerance);
    EXPECT_NEAR(_joint->LowerLimit(_index), limit, g_tolerance);
    {
      boost::any value = _joint->GetParam("lo_stop", _index);
      EXPECT_NEAR(boost::any_cast<double>(value), limit, g_tolerance);
    }
  }
}

TEST_P(JointSpawningTest_All, SpawnJointTypes)
{
  if (this->jointType == "gearbox" && this->physicsEngine != "ode")
  {
    gzerr << "Skip test, gearbox is only supported in ODE. " <<
         "See issues: #859, #1914, #1915" << std::endl;
    return;
  }
  if (physicsEngine == "simbody" && jointType == "revolute2")
  {
    gzerr << "Skip test, revolute2 not supported in simbody, see issue #859."
          << std::endl;
    return;
  }
  SpawnJointTypes(this->physicsEngine, this->jointType);
}

TEST_P(JointSpawningTest_Rotational, SpawnJointRotational)
{
  SpawnJointRotational(this->physicsEngine, this->jointType);
}

TEST_P(JointSpawningTest_RotationalWorld, SpawnJointRotationalWorld)
{
  SpawnJointRotationalWorld(this->physicsEngine, this->jointType);
}

INSTANTIATE_TEST_CASE_P(TestRuns, JointSpawningTest_All,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("revolute"
                  , "prismatic"
                  , "screw"
                  , "universal"
                  , "fixed"
                  , "ball"
                  , "revolute2"
                  , "gearbox")));

// Skip prismatic, screw, and revolute2 because they allow translation
INSTANTIATE_TEST_CASE_P(TestRuns, JointSpawningTest_Rotational,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("revolute"
                  , "universal"
                  , "fixed"
                  , "ball")));

// Skip prismatic, screw, and revolute2 because they allow translation
INSTANTIATE_TEST_CASE_P(TestRuns, JointSpawningTest_RotationalWorld,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("revolute"
                  , "universal"
                  , "fixed"
                  , "ball")));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
