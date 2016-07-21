/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
// #include "gazebo/physics/Joint.hh"
// #include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

using namespace gazebo;
const double g_tolerance = 1e-2;

class JointTestScrew : public JointTest
{
  /// \brief Test screw joint implementation with SetWorldPose.
  /// Set link poses in world frame, check joint angles and joint axis.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void ScrewJointSetWorldPose(const std::string &_physicsEngine);

  /// \brief Test screw joint implementation with forces.
  /// Apply force to screw joint links, check velocity.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void ScrewJointForce(const std::string &_physicsEngine);

  /// \brief Test screw joint limits implementation with forces.
  /// Apply force to screw joint links, check velocity.
  /// Keep increasing force until something gives
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void ScrewJointLimitForce(const std::string &_physicsEngine);

  /// \brief Spin joints several rotations and verify that the angles
  /// wrap properly.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void WrapAngle(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////
void JointTestScrew::WrapAngle(const std::string &_physicsEngine)
{
  /// \TODO: bullet hinge angles are wrapped (#1074)
  if (_physicsEngine == "bullet")
  {
    gzerr << "Aborting test for bullet, see issues #1074.\n";
    return;
  }

  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->Type(), _physicsEngine);

  // disable gravity
  physics->SetGravity(ignition::math::Vector3d::Zero);

  {
    std::string jointType = "screw";
    gzdbg << "SpawnJoint " << jointType << " child world" << std::endl;
    physics::JointPtr joint = SpawnJoint(jointType, false, true);
    ASSERT_TRUE(joint != NULL);

    // \TODO: option to set thread pitch, create another test
    // double threadPitch = 100.0;
    // joint->SetParam("thread_pitch", threadPitch);

    // Inertial parameters
    const double momentOfInertia = 1.0;
    const double threadPitch = 1.0;
    const double mass = 1.0;
    double inertia = momentOfInertia + mass / (threadPitch*threadPitch);

    // Verify inertial parameters
    EXPECT_NEAR(threadPitch, joint->Param("thread_pitch", 0), g_tolerance);
    {
      physics::LinkPtr child = joint->Child();
      EXPECT_NEAR(mass, child->Inertia().Mass(), g_tolerance);
    }
    /// \TODO: verify momentOfInertia

    // set torque and step forward
    const double torque = 35;
    const unsigned int stepCount = 1000;
    double dt = physics->MaxStepSize();
    double stepTime = stepCount * dt;

    // Expect constant torque to give quadratic response in position
    {
      // Expected max joint angle (quatratic in time)
      ignition::math::Angle maxAngle(
          0.5 * torque * stepTime*stepTime / inertia);
      // Verify that the joint should make more than 1 revolution
      EXPECT_GT(maxAngle.Radian(), 1.25 * 2 * M_PI);
    }

    // compute joint velocity analytically with constant torque
    // joint angle is unwrapped
    for (unsigned int i = 0; i < stepCount; ++i)
    {
      joint->SetForce(0, torque);

      double vel = sqrt(2.0*torque*joint->Angle(0).Radian() / inertia);
      world->Step(1);
      EXPECT_NEAR(joint->Velocity(0), vel, 2e-2);
      double time = world->SimTime().Double();
      ignition::math::Angle angle(0.5 * torque * time*time / inertia);
      EXPECT_NEAR(joint->Angle(0).Radian(), angle.Radian(), g_tolerance);
    }
    std::cout << "Final time:  " << world->SimTime().Double() << std::endl;
    std::cout << "Final angle: " << joint->Angle(0).Radian() << std::endl;
    std::cout << "Final speed: " << joint->Velocity(0) << std::endl;
  }
}

TEST_P(JointTestScrew, WrapAngle)
{
  WrapAngle(this->physicsEngine);
}

//////////////////////////////////////////////////
void JointTestScrew::ScrewJointSetWorldPose(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "DART Screw Joint will not work with Link::SetWorldPose."
          << " See issue #1096.\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Screw Joint will not work with Link::SetWorldPose."
          << " See issue #857.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/screw_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->Type(), _physicsEngine);

  physics->SetGravity(ignition::math::Vector3d(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->SimTime().Double();

  // get time step size
  double dt = world->Physics()->MaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get model, joints and get links
  physics::ModelPtr model1 = world->ModelByName("model1");
  physics::LinkPtr link00 = model1->LinkByName("link00");
  physics::LinkPtr link01 = model1->LinkByName("link01");
  physics::JointPtr joint00 = model1->JointByName("joint00");
  physics::JointPtr joint01 = model1->JointByName("joint01");

  // both initial angles should be zero
  EXPECT_EQ(joint00->Angle(0), 0);
  EXPECT_EQ(joint00->Angle(1), 0);

  // move child link to it's initial location
  link00->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0, 0, 0));
  EXPECT_EQ(joint00->Angle(0), 0);
  EXPECT_EQ(joint00->Angle(1), 0);
  EXPECT_EQ(joint00->GlobalAxis(0), ignition::math::Vector3d(1, 0, 0));
  EXPECT_EQ(joint00->GlobalAxis(1), ignition::math::Vector3d(1, 0, 0));
  gzdbg << "joint angles [" << joint00->Angle(0)
        << ", " << joint00->Angle(1)
        << "] axis1 [" << joint00->GlobalAxis(0)
        << "] axis2 [" << joint00->GlobalAxis(1)
        << "]\n";

  // move child link 45deg about x
  double pitch00 = joint00->Param("thread_pitch", 0);
  ignition::math::Pose3d pose00 =
    ignition::math::Pose3d(-0.25*M_PI/pitch00, 0, 2, 0.25*M_PI, 0, 0);
  ignition::math::Pose3d pose01 = ignition::math::Pose3d(0, 0, -1, 0, 0, 0) +
    pose00;
  link00->SetWorldPose(pose00);
  link01->SetWorldPose(pose01);
  EXPECT_EQ(joint00->Angle(0), 0.25*M_PI);
  EXPECT_EQ(joint00->Angle(1), -0.25*M_PI/pitch00);
  EXPECT_EQ(joint00->GlobalAxis(0), ignition::math::Vector3d(1, 0, 0));
  EXPECT_EQ(joint00->GlobalAxis(1), ignition::math::Vector3d(1, 0, 0));
  gzdbg << "joint angles [" << joint00->Angle(0)
        << ", " << joint00->Angle(1)
        << "] axis1 [" << joint00->GlobalAxis(0)
        << "] axis2 [" << joint00->GlobalAxis(1)
        << "] pitch00 [" << pitch00
        << "]\n";

  // move child link 45deg about y
  double pitch01 = joint01->Param("thread_pitch", 0);
  link00->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0, 0.25*M_PI, 0));
  pose00 = ignition::math::Pose3d(-0.25*M_PI/pitch00, 0, 2, 0.25*M_PI, 0, 0);
  pose01 = ignition::math::Pose3d(-0.3*M_PI/pitch01, 0, -1, 0.3*M_PI, 0, 0) +
    pose00;
  link00->SetWorldPose(pose00);
  link01->SetWorldPose(pose01);
  EXPECT_EQ(joint00->Angle(0), 0.25*M_PI);
  EXPECT_EQ(joint00->Angle(1), -0.25*M_PI/pitch00);
  EXPECT_EQ(joint01->Angle(0), 0.3*M_PI);
  EXPECT_EQ(joint01->Angle(1), -0.3*M_PI/pitch01);
  EXPECT_EQ(joint00->GlobalAxis(0), ignition::math::Vector3d(1, 0, 0));
  EXPECT_EQ(joint00->GlobalAxis(1), ignition::math::Vector3d(1, 0, 0));
  gzdbg << "joint angles [" << joint00->Angle(0)
        << ", " << joint00->Angle(1)
        << "] axis1 [" << joint00->GlobalAxis(0)
        << "] axis2 [" << joint00->GlobalAxis(1)
        << "] pitch00 [" << pitch00
        << "] pitch01 [" << pitch01
        << "]\n";

  // new poses should not violate the constraint.  take a few steps
  // and make sure nothing moves.
  world->Step(10);

  // move child link 90deg about both x and "rotated y axis" (z)
  EXPECT_EQ(joint00->Angle(0), 0.25*M_PI);
  EXPECT_EQ(joint00->Angle(1), -0.25*M_PI/pitch00);
  EXPECT_EQ(joint01->Angle(0), 0.3*M_PI);
  EXPECT_EQ(joint01->Angle(1), -0.3*M_PI/pitch01);
  EXPECT_EQ(joint00->GlobalAxis(0), ignition::math::Vector3d(1, 0, 0));
  EXPECT_EQ(joint00->GlobalAxis(1), ignition::math::Vector3d(1, 0, 0));
  gzdbg << "joint angles [" << joint00->Angle(0)
        << ", " << joint00->Angle(1)
        << "] axis1 [" << joint00->GlobalAxis(0)
        << "] axis2 [" << joint00->GlobalAxis(1)
        << "] pitch00 [" << pitch00
        << "] pitch01 [" << pitch01
        << "]\n";
}

TEST_P(JointTestScrew, ScrewJointSetWorldPose)
{
  ScrewJointSetWorldPose(this->physicsEngine);
}

//////////////////////////////////////////////////
void JointTestScrew::ScrewJointForce(const std::string &_physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    /// \TODO skipping bullet, see issue #1081
    gzerr << "BulletScrewJoint::Angle() is one step behind (issue #1081).\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/screw_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->Type(), _physicsEngine);

  physics->SetGravity(ignition::math::Vector3d(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->SimTime().Double();

  // get time step size
  double dt = world->Physics()->MaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get model, joints and get links
  physics::ModelPtr model1 = world->ModelByName("model1");
  physics::LinkPtr link00 = model1->LinkByName("link00");
  physics::LinkPtr link01 = model1->LinkByName("link01");
  physics::JointPtr joint00 = model1->JointByName("joint00");
  physics::JointPtr joint01 = model1->JointByName("joint01");
  double pitch00 = joint00->Param("thread_pitch", 0);
  double pitch01 = joint01->Param("thread_pitch", 0);

  // both initial angles should be zero
  EXPECT_EQ(joint00->Angle(0), 0);
  EXPECT_EQ(joint00->Angle(1), 0);

  // set new upper limit for joint00
  joint00->SetHighStop(0, ignition::math::Angle(0.3));
  bool once = false;
  int count = 0;
  int maxCount = 5000;
  // push joint00 till it hits new upper limit
  while (count < maxCount && joint00->Angle(0) < 0.3)
  {
    joint00->SetForce(0, 0.1);
    world->Step(1);
    ++count;
    // check link pose
    double angle00_angular = joint00->Angle(0).Radian();
    double angle00_linear = joint00->Angle(1).Radian();
    double angle01_linear = joint01->Angle(1).Radian();
    ignition::math::Pose3d pose01 = link01->WorldPose();
    EXPECT_EQ(link00->WorldPose(),
      ignition::math::Pose3d(-angle00_angular / pitch00, 0, 2,
        angle00_angular, 0, 0));

    if (_physicsEngine == "simbody")
    {
      if (!once)
      {
        once = true;
        gzerr << "skip test: issue #857 in simbody screw joint linear angle:"
              << " joint00 " << angle00_linear
              << " shoudl be 0.3\n";
      }
    }
    else
    {
      EXPECT_NEAR(pose01.Pos().X(), angle00_linear + angle01_linear, 1e-8);
    }
  }
  gzdbg << "took [" << count << "] steps.\n";

  // continue pushing for 1000 steps to make sure there is no overshoot
  double maxOvershootRadians = 0.01;
  for (unsigned int i = 0; i < 1000; ++i)
  {
    joint00->SetForce(0, 0.1);
    world->Step(1);
    EXPECT_LT(joint00->Angle(0), 0.3 + maxOvershootRadians);
  }


  // lock joint at this location by setting lower limit here too
  joint00->SetLowStop(0, ignition::math::Angle(0.3));

  // set joint01 upper limit to 1.0
  joint01->SetHighStop(0, ignition::math::Angle(1.0));

  // push joint01 until limit is reached
  once = false;
  count = 0;
  while (count < maxCount && joint01->Angle(0) < 1.0)
  {
    joint01->SetForce(0, 0.1);
    world->Step(1);
    ++count;

    // check link pose
    ignition::math::Pose3d pose00 = link00->WorldPose();
    ignition::math::Pose3d pose01 = link01->WorldPose();
    double angle00_angular = joint00->Angle(0).Radian();
    double angle00_linear = joint00->Angle(1).Radian();
    double angle01_angular = joint01->Angle(0).Radian();
    double angle01_linear = joint01->Angle(1).Radian();

    EXPECT_EQ(pose00, ignition::math::Pose3d(
      -angle00_angular / pitch00, 0, 2, angle00_angular, 0, 0));
    if (_physicsEngine == "simbody")
    {
      if (!once)
      {
        once = true;
        gzerr << "skip test: issue #857 in simbody screw joint linear angle:"
              << " joint00 " << angle00_linear
              << " should be 0.3. "
              << " joint01 " << angle01_linear
              << " is off too.\n";
      }
    }
    else
    {
      EXPECT_NEAR(pose01.Pos().X(), angle00_linear + angle01_linear, 1e-8);
    }
    EXPECT_NEAR(pose01.Pos().X(),
      -angle00_angular / pitch00 - angle01_angular / pitch01, 1e-8);
    EXPECT_NEAR(pose01.Rot().Euler().X(),
      angle00_angular + angle01_angular, 1e-8);
  }
  gzdbg << "took [" << count << "] steps.\n";

  // continue pushing for 1000 steps to make sure there is no overshoot
  for (unsigned int i = 0; i < 1000; ++i)
  {
    joint01->SetForce(0, 0.1);
    world->Step(1);
    EXPECT_LT(joint01->Angle(0), 1.0 + maxOvershootRadians);
  }

  // push joint01 the other way until -1 is reached
  once = false;
  count = 0;
  while (count < maxCount && joint01->Angle(0) > -1.0)
  {
    joint01->SetForce(0, -0.1);
    world->Step(1);
    ++count;

    // check link pose
    ignition::math::Pose3d pose00 = link00->WorldPose();
    ignition::math::Pose3d pose01 = link01->WorldPose();
    double angle00_angular = joint00->Angle(0).Radian();
    double angle00_linear = joint00->Angle(1).Radian();
    double angle01_angular = joint01->Angle(0).Radian();
    double angle01_linear = joint01->Angle(1).Radian();

    EXPECT_EQ(pose00, ignition::math::Pose3d(
      -angle00_angular / pitch00, 0, 2, angle00_angular, 0, 0));
    if (_physicsEngine == "simbody")
    {
      if (!once)
      {
        once = true;
        gzerr << "skip test: issue #857 in simbody screw joint linear angle:"
              << " joint00 " << angle00_linear
              << " should be 0.3. "
              << " joint01 " << angle01_linear
              << " is off too.\n";
      }
    }
    else
    {
      EXPECT_NEAR(pose01.Pos().X(), angle00_linear + angle01_linear, 1e-8);
    }
    EXPECT_NEAR(pose01.Pos().X(),
      -angle00_angular / pitch00 - angle01_angular / pitch01, 1e-8);
    EXPECT_NEAR(pose01.Rot().Euler().X(),
      angle00_angular + angle01_angular, 1e-8);
  }
  gzdbg << "took [" << count << "] steps.\n";

  // continue pushing for 1000 steps to make sure there is no overshoot
  joint01->SetLowStop(0, ignition::math::Angle(-1.0));
  for (unsigned int i = 0; i < 1000; ++i)
  {
    joint01->SetForce(0, -0.1);
    world->Step(1);
    EXPECT_GT(joint01->Angle(0), -1.0 - maxOvershootRadians);
  }
}

TEST_P(JointTestScrew, ScrewJointForce)
{
  ScrewJointForce(this->physicsEngine);
}

//////////////////////////////////////////////////
void JointTestScrew::ScrewJointLimitForce(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << _physicsEngine
          << " is broken for this test,"
          << " because of the pr2 gripper's closed kinematic chain,"
          << " see issues #1435 and #1933."
          << std::endl;
    return;
  }

  // Load pr2 world
  ServerFixture::Load("worlds/pr2.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->Type(), _physicsEngine);

  // get time step size
  double dt = world->Physics()->MaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // get model, joints and get links
  physics::ModelPtr model = world->ModelByName("pr2");
  physics::LinkPtr link00 = model->LinkByName("torso_lift_link");

  // drop from some height
  model->SetWorldPose(ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));
  // +1sec: should have hit the ground
  world->Step(1000);
  // +4sec: should destabilize without patch for issue #1159
  world->Step(4000);

  for (int n = 0; n < 1000; ++n)
  {
    world->Step(1);
    ignition::math::Vector3d vel_angular = link00->WorldLinearVel();
    ignition::math::Vector3d vel_linear = link00->WorldAngularVel();

    EXPECT_LT(vel_angular.Length(), 0.1);
    EXPECT_LT(vel_linear.Length(), 0.1);

    gzlog <<   "va [" << vel_angular
          << "] vl [" << vel_linear
          << "]\n";
  }
}

TEST_P(JointTestScrew, ScrewJointLimitForce)
{
  ScrewJointLimitForce(this->physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointTestScrew,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("screw")));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
